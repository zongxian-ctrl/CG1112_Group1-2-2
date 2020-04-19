//forward @ 100%
//backwards @


#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include <avr/sleep.h>

#include "packet.h"
#include "constants.h"

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV      170
#define PRR_TWI_MASK 0b10000000
#define PRR_SPI_MASK 0b00000100
#define ADCSRA_ADC_MASK 0b10000000
#define PRR_ADC_MASK 0b00000001
#define PRR_TIMER2_MASK 0b01000000
#define PRR_TIMER0_MASK 0b00100000
#define PRR_TIMER1_MASK 0b00001000
#define SMCR_SLEEP_ENABLE_MASK 0b00000001
#define SMCR_IDLE_MODE_MASK 0b11110001
// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.4

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin
/*
 *    Alex's State Variables
 */

/*// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftTicks;
volatile unsigned long rightTicks;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;*/

// PI, for calculating turn circumference
#define PIE          3.141592654

// Alex's length and breath in cm
#define ALEX_LENGTH  19.5
#define ALEX_BREADTH  12.5

// Alex's diagonal. We compute and store this once
// since it is expensive to compute and really doesn't change.
float alexDiagonal = 0.0;

 // Alex's turning circumference, calculate once
float alexCirc = 0.0;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variables to keep track of whether we have moved a command distance
unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

//Forward/backward
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

//TURNING
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

/*
 *
 * Alex Communication Routines.
 *
 */

TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".

    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);

}

void WDT_off(void)
{
  /* Global interrupt should be turned OFF here if not
  already done so */
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1<<WDRF);
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional
  time-out */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  /* Turn off WDT */
  WDTCSR = 0x00;
}

// For Power Management 
void setupPowerSaving()
{
  // Turn off the Watchdog Timer
  WDT_off();
  // Modify PRR to shut down TWI
  // Modify PRR to shut down SPI
  PRR |= PRR_TWI_MASK;
  PRR |= PRR_SPI_MASK;
  // Modify ADCSRA to disable ADC,
  ADCSRA &= ~ADCSRA_ADC_MASK;
  // then modify PRR to shut down ADC
  PRR |= PRR_ADC_MASK;
  // Set the SMCR to choose the IDLE sleep mode
  SMCR &= SMCR_IDLE_MODE_MASK;
  // Do not set the Sleep Enable (SE) bit yet
  // Set Port B Pin 5 as output pin, then write a logic LOW
  // to it so that the LED tied to Arduino's Pin 13 is OFF.
  DDRB |= 0b00100000;
  PORTB &= 0b11011111;
}

void putArduinoToIdle()
{
  // Modify PRR to shut down TIMER 0, 1, and 2
  PRR |= (PRR_TIMER2_MASK | PRR_TIMER0_MASK | PRR_TIMER1_MASK);
  // Modify SE bit in SMCR to enable (i.e., allow) sleep
  SMCR |= SMCR_SLEEP_ENABLE_MASK;
  // The following function puts ATmega328P’s MCU into sleep;
  // it wakes up from sleep when USART serial data arrives
  sleep_cpu();
  // Modify SE bit in SMCR to disable (i.e., disallow) sleep
  SMCR &= ~SMCR_SLEEP_ENABLE_MASK;
  // Modify PRR to power up TIMER 0, 1, and 2
  PRR &= ~(PRR_TIMER2_MASK | PRR_TIMER0_MASK | PRR_TIMER1_MASK);
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.

  TPacket statusPacket;
  statusPacket.packetType=PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;

  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;

  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;

  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;

  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...)
{
  va_list args;
  char buffer [128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);

}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and
 * pullup resistors.
 *
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011;

  PORTD |= 0b00001100;

}

// Functions to be called by INT0 and INT1 ISRs.
void LEFTISR() //LEFTISR
{
  switch(dir)
  {
    case FORWARD:
      leftForwardTicks++;
      break;

    case BACKWARD:
      leftReverseTicks++;
      break;

    case LEFT:
      leftReverseTicksTurns++;
      break;

    case RIGHT:
      leftForwardTicksTurns++;
      break;
  }

  if(dir == FORWARD)
  {
    forwardDist = (unsigned long)((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  if(dir == BACKWARD)
  {
    reverseDist = (unsigned long)((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
}

ISR(INT0_vect)
{
  LEFTISR();
}

void RIGHTISR() //RIGHTISR
{
  switch(dir)
  {
    case FORWARD:
      rightForwardTicks++;
      break;

    case BACKWARD:
      rightReverseTicks++;
      break;

    case RIGHT:
      rightReverseTicksTurns++;
      break;

    case LEFT:
      rightForwardTicksTurns++;
      break;
  }
}

ISR(INT1_vect)
{
  RIGHTISR();
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EICRA = 0b00001010;
  EIMSK = 0b00000011;

}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.




// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 *
 */
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
	//Serial.begin(9600);
	Serial.begin(57600);

  // Tried Bare Metal but failed
	/*unsigned int b;
  b = round(16000000/(16.0 * 57600)) – 1;
  UBRR0H = b >> 8;
	UBRR0L = b;
  UCSR0C = 0b00000110;
  UCSR0A = 0;*/

}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Tried Bare Metal but failed
  /*UCSR0B = 0b00011000;*/
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.

int readSerial(char *buffer)
{
  int count=0;

	while(Serial.available())
		buffer[count++] = Serial.read();

  // Tried Bare Metal but failed
	/*while((UCSR0A & 0b00100000) == 1)
	buffer[count++] = UDR0;*/

	return count;
}

// Write to the serial port.

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);

  // Tried Bare Metal but failed
	/*for(int count = 0 ; count < sizeof(buffer) ; count++)
  {
	while( (UCSR0A & 0b00100000) == 0)
		UDR0 = buffer[count];
  }*/

}

/*
 * Alex's motor drivers.
 *
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */

   // Tried Bare Metal but failed
   /*DDRB |= (RF | RR);
   DDRD |= (LF | LR);

   TCNT0 = 0;
   OCR0A = 0;
   OCR0B = 0;
   TIMSK0 |= 0b110; // OCIEA = 1 OCIEB = 1
   TCCR0B = 0b00000011;
   TCNT1 = 0;
   OCR1A = 0;
   OCR1B = 0;
   TIMSK1 |= 0b110; // OCIEA = 1 OCIEB = 1
   TCCR1B = 0b00000011;*/
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{

}

// Tried Bare Metal but failed
/*static volatile int LFval;
static volatile int LRval;
static volatile int RFval;
static volatile int RRval;
static volatile int LFvalinit;
static volatile int LRvalinit;
static volatile int RFvalinit;
static volatile int RRvalinit;

ISR(TIMER0_COMPA_vect)
{
	OCR0A = LRval;
}

ISR(TIMER0_COMPB_vect)
{
	OCR0B = LFval;
}
ISR(TIMER1_COMPA_vect)
{
	OCR1A = RFval;
}

ISR(TIMER1_COMPB_vect)
{
	OCR1B = RRval;
}*/

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  dir = FORWARD;

  int val = pwmVal(speed);

  //Code to tell us how far to move
  if(dist == 0)
    deltaDist = 999999;
  else
    deltaDist = dist;

  newDist = forwardDist + deltaDist;

  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.

  analogWrite(LF, val);
  analogWrite(RF, val);
  analogWrite(LR,0);
  analogWrite(RR, 0);

  // Tried Bare Metal but failed
  /*TCCR0A = 0b00100001;
  PORTD &= ~LR; //off LR
  LFval = val;
  LFvalinit = LFval;

  TCCR1A = 0b10000001;
  PORTB &= ~RR; //off RR
  RFval = val;
  RFvalinit = RFval;*/
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  dir = BACKWARD;

  int val = pwmVal(speed);

  //Code to tell us how far to move
  if(dist == 0)
    deltaDist = 999999;
  else
    deltaDist = dist;

  newDist = reverseDist + deltaDist;

  // For now we will ignore dist and
  // reverse indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  analogWrite(LR, val);
  analogWrite(RR, val);
  analogWrite(LF, 0);
  analogWrite(RF, 0);

  // Tried Bare Metal but failed
  /*TCCR0A = 0b10000001;
  PORTD &= ~LF; //off LF
  LRval = val;
  LRvalinit = LRval;

  TCCR1A = 0b00100001;
  PORTB &= ~RF; //off RF
  RRval = val;
  RRvalinit = RRval;*/
}

unsigned long computeDeltaTicks(float ang)
{
  // We will assume that anguar distance moved = linear distance moved in one wheel
  // revolution. This is (probably) incorrect but simplifies calculation.
  // # of wheel revs to make one full 360 turn is alexCirc / WHEEL_CIRC
  // This is for 360 degrees. For ang degrees it will be (ang * alexCirc) / (360 * WHEEL_CIRC)
  // To convert to Ticks, we multiply by COUNTS_PER_REV.

  unsigned long ticks = (unsigned long) ((ang * alexCirc * (136.0/2)) / (360.0 * WHEEL_CIRC));

  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;

  int val = pwmVal(speed);

  if(ang == 0) {
    deltaTicks = 999999;
  }
  else{
    deltaTicks = computeDeltaTicks(ang);
  }

  targetTicks = leftReverseTicksTurns + deltaTicks;

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  analogWrite(LR, val+25); // Left-Heavy, increase power
  analogWrite(RF, val);
  analogWrite(LF, 0);
  analogWrite(RR, 0);

  // Tried Bare Metal but failed
  /*TCCR0A = 0b10000001;
  PORTD &= ~LF; //off LF
  LRval = val+25;
  LRvalinit = LRval;

  TCCR1A = 0b10000001;
  PORTB &= ~RR; //off RR
  RFval = val;
  RFvalinit = RFval;*/
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;

  int val = pwmVal(speed);

  if(ang == 0){
    deltaTicks = 999999;
  }
  else{
    deltaTicks = computeDeltaTicks(ang);
  }

  targetTicks = rightReverseTicksTurns + deltaTicks;

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  analogWrite(RR, val);
  analogWrite(LF, val+25); // Left-Heavy, increase power
  analogWrite(LR, 0);
  analogWrite(RF, 0);

  // Tried Bare Metal but failed
  /*TCCR0A = 0b00100001;
  PORTD &= ~LR; //off LR
  LFval = val + 20;
  LFvalinit = LFval;

  TCCR1A = 0b00100001;
  PORTB &= ~RF; //off RF
  RRval = val;
  RRvalinit = RRval;*/
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;

  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);

  // Tried Bare Metal but failed
  /*TCCR0A = 0b00000001;
  PORTD &= (~LR & ~LF); //off LR & LF

  TCCR1A = 0b00000001;
  PORTB &= ( ~RR & ~ RF ); //off RF & RF*/
}

/*
 * Alex's setup and run codes
 *
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;

  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;

  forwardDist = 0;
  reverseDist = 0;

}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_STOP:
        sendOK();
        stop();
      break;
    case COMMAND_GET_STATS:
        sendStatus();
      break;
    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]);
      break;

    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {


        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:

  // Compute the diagonal
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));

  alexCirc = PIE * alexDiagonal;



  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  setupPowerSaving();
  sei();

  //Tried Bare Metal, but failed
  /*DDRB &= 0b11000111; //using pin 11,12,13*/
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

  if(deltaDist > 0)
  {
    if(dir == FORWARD)
    {
      if(forwardDist >= newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if(dir == BACKWARD)
    {
      if(reverseDist >= newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if(dir == STOP)
    {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if(deltaTicks > 0){
    if(dir == LEFT)
    {
      if(leftReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }

    }
    else if(dir == RIGHT)
    {
      if(rightReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if(dir == STOP)
    {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
  }
// Uncomment the code below for Week 9 Studio 2


// put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi
  if(dir == STOP)
    putArduinoToIdle();
  TResult result = readPacket(&recvPacket);

  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      }


}
