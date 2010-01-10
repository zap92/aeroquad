// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this

// Created 29 March 2006
// I2C Konstanten
#define SCL_CLOCK  200000L
#define I2C_TIMEOUT 30000
#define I2C_START          0x08
#define I2C_REPEATED_START 0x10
#define I2C_TX_SLA_ACK     0x18
#define I2C_TX_DATA_ACK    0x28
#define I2C_RX_SLA_ACK     0x40
#define I2C_RX_DATA_ACK    0x50
#define MAX_MOTORS 6
#define CPU_FREQ 16000000L
#define TWI_FREQ 200000L

//############################################################################
//#include <Wire.h>

#include "WProgram.h"
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_reset(void);
void i2c_write_byte(char byte);
void I2C_WriteByte(int8_t byte);
void I2C_ReceiveByte(void);
void I2C_ReceiveLastByte(void);
void setup();
void loop();
volatile unsigned char twi_state = 0;
unsigned char Motor[MAX_MOTORS];
unsigned char motor=0,PlatinenVersion=0;
unsigned char motors=0;
unsigned char motorread = 0,MissingMotor = 0;
unsigned char motor_rx[16],motor_rx2[16];
unsigned char MotorPresent[MAX_MOTORS];
unsigned char MotorError[MAX_MOTORS];
unsigned int I2CTimeout=100;
unsigned int I2CError = 0;
struct
  {
    char Revision;
    char Name[12];
    signed char Motor[16][4];
  } Mixer;

byte x = 0;
int mnumber = 6;
byte command=0;


//############################################################################
//Initzialisieren der I2C (TWI) Schnittstelle
void i2c_init(void)
//############################################################################
{
  TWSR = 0;
  TWBR = ((CPU_FREQ/TWI_FREQ)-16)/2; 
}

//############################################################################
//Start I2C
void i2c_start(void) 
//############################################################################
{
    TWCR = (1<<TWSTA) | (1<<TWEN) | (1<<TWINT) | (1<<TWIE);
}

//############################################################################
void i2c_stop(void)
//############################################################################
{
    TWCR = (1<<TWEN) | (1<<TWSTO) | (1<<TWINT);
}

void i2c_reset(void)
//############################################################################
{
                 i2c_stop();                
                 twi_state = 0;
                 motor = TWDR;
                 motor = 0;
                 TWCR = 0x80;
                 TWAMR = 0;
                 TWAR = 0;
                 TWDR = 0;
                 TWSR = 0;
                 TWBR = 0;
                 i2c_init();
                 i2c_start();
                 i2c_write_byte(0);
}

//############################################################################
void i2c_write_byte(char byte)
//############################################################################
{ 
    TWSR = 0x00;
    TWDR = byte;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
}

/****************************************/
/*    Write to I2C                      */
/****************************************/
void I2C_WriteByte(int8_t byte)
{
    // move byte to send into TWI Data Register
    TWDR = byte;
    // clear interrupt flag (TWINT = 1)
    // enable i2c bus (TWEN = 1)
    // enable interrupt (TWIE = 1)
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
}

/****************************************/
/*    Receive byte and send ACK         */
/****************************************/
void I2C_ReceiveByte(void)
{
   TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
}

/****************************************/
/* I2C receive last byte and send no ACK*/
/****************************************/
void I2C_ReceiveLastByte(void)
{
   TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
}



//############################################################################
SIGNAL (TWI_vect)
//############################################################################
{
 static unsigned char missing_motor;
     Serial.print("TWI_VECT");
     switch(twi_state++)
        {
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Writing the Data
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        case 0:
		        
                     // while( motor < MAX_MOTORS) motor++;  // skip if not used
				if(motor == MAX_MOTORS)  // writing finished -> now read 
				{ 
                                motor = 0; 
				 twi_state = 3; 
				 i2c_write_byte(0x53+(motorread*2));
				 Serial.print("I2C ADDR MAX");
				 } 
				else 
                                {
                                i2c_write_byte(0x52+(motor*2));
                                Serial.print("I2C ADDR");
                                }
                                
                break;
        case 1:
                i2c_write_byte(Motor[motor++]);
                Serial.print("I2C PWR LEVEL");
                break;
        case 2:
                if(TWSR == 0x30) 
				 { 
				  if(!missing_motor) missing_motor = motor; 
				  if(++MotorError[motor-1] == 0) MotorError[motor-1] = 255;
				 }
                i2c_stop();
                Serial.print("I2C STOP");
 
                I2CTimeout = 10;
                twi_state = 0;
                i2c_start();   
                break; 
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Reading Data
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        case 3:
               //Transmit 1st byte for reading
                if(TWSR != 0x40)  // Error?
                 { 
                  MotorPresent[motorread] = 0;
                  motorread++; 
                  if(motorread >= MAX_MOTORS) motorread = 0;
                  i2c_stop();
                  twi_state = 0;
                 }
                 else 
                 {
                  MotorPresent[motorread] = ('1' - '-') + motorread;
                  I2C_ReceiveByte();
                 }
                MissingMotor = missing_motor;
                missing_motor = 0;
                break;
        case 4: //Read 1st byte and transmit 2nd Byte
                motor_rx[motorread] = TWDR;
                I2C_ReceiveLastByte(); //nack
                break;
        case 5:
                //Read 2nd byte
                motor_rx2[motorread++] = TWDR;
                if(motorread >= MAX_MOTORS) motorread = 0;
                i2c_stop();
                twi_state = 0;
                break;
	default: twi_state = 0;
                break;        
		}
 TWCR |= 0x80;
}


void setup()
{
  	unsigned int timer,i,timer2 = 0;
    DDRB  = 0x00;
    PORTB = 0x00;
    for(timer = 0; timer < 1000; timer++); // verz\u00f6gern
    if(PINB & 0x01)
     {
      if(PINB & 0x02) PlatinenVersion = 13;
       else           PlatinenVersion = 11;
     }
    else
     {
      if(PINB & 0x02) PlatinenVersion = 20;
       else           PlatinenVersion = 10;
     }

    DDRC  = 0x81; // SCL
    DDRC  |=0x40; // HEF4017 Reset
    PORTC = 0xff; // Pullup SDA
    DDRB  = 0x1B; // LEDs und Druckoffset
    PORTB = 0x01; // LED_Rot
    DDRD  = 0x3E; // Speaker & TXD & J3 J4 J5
	PORTD = 0x47; // LED
 //   HEF4017R_ON;
    MCUSR &=~(1<<WDRF);
    WDTCSR |= (1<<WDCE)|(1<<WDE);
    WDTCSR = 0;
    PORTD=0xff;
  Serial.begin(115200);
  Serial.print("Test Motori I2C");
  Serial.println();
  i2c_reset();
  i2c_init();
  sei();
  motor=0;
}

void loop()
{

    //Start I2C Interrupt Mode
/*
  Serial.print("Spengo i motori");
  for(mnumber=1;mnumber<7;mnumber++)
  {
    Motor[mnumber]=0;
    twi_state = 0;
    motor = 0;
    i2c_start();
    delay(20);
  }  
  i2c_start();
*/  
  Serial.print("Accendo i motori");
    Serial.print(motor);
    for(x=0;x<230;x++)
    {
    for(mnumber=1;mnumber<7;mnumber++)
       {
        Motor[mnumber]=x;
                
        }
        Serial.print("I2CStart");
        twi_state = 0;
        motor = 0;
        i2c_start(); 
        Serial.println();
        delay(100);  
     }
  
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

