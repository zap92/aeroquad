/*
  AeroQuad v1.5 - Novmeber 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/


#include "Motors.h"

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

volatile unsigned char twi_state = 0;
unsigned char Motor[MAX_MOTORS];
unsigned char i2cmotor=0,PlatinenVersion=0;
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
  //TWBR = ((CPU_FREQ/TWI_FREQ)-16)/2; 
  TWBR = ((CPU_FREQ/TWI_FREQ)-16)/2;
  TWBR = 0x15;
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
                 i2cmotor = TWDR;
                 i2cmotor = 0;
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
void i2c_write_byte(char i2cbyte)
//############################################################################
{ 
    TWSR = 0x00;
    TWDR = i2cbyte;
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
}

/****************************************/
/*    Write to I2C                      */
/****************************************/
void I2C_WriteByte(int8_t i2cbyte)
{
    // move byte to send into TWI Data Register
    TWDR = i2cbyte;
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
    //Serial.print("TWI_VECT");
     switch(twi_state++)
        {
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Writing the Data
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        case 0:
                                  //i2cmotor++;  // skip if not used		  
                i2c_write_byte(0x52+(i2cmotor*2));
                //Serial.print("I2C ADDR");
                //Serial.print(i2cmotor);
                //Serial.println("");
                                  
                                
                break;
        case 1:
                switch(i2cmotor++)
                { 
                  
                case 0:
                i2c_write_byte(Motor[FRONTMOTORID]);
                break;
                case 1:
                i2c_write_byte(Motor[REARMOTORID]);
                break;
                case 2:
                i2c_write_byte(Motor[RIGHTMOTORID]);
                break;
                case 3:
                i2c_write_byte(Motor[LEFTMOTORID]);                
                break;
                default:
                break;
                }
                //Serial.print("I2C PWR LEVEL");
                //Serial.print(Motor[i2cmotor]);
                //Serial.println();
                break;
        case 2:
              //  if(TWSR == 0x30) 
	      //			 { 
	      //			  if(!missing_motor) missing_motor = i2cmotor; 
	      //			  if(++MotorError[i2cmotor-1] == 0) MotorError[i2cmotor-1] = 255;
	      //			 }
                i2c_stop();
                //Serial.println("I2C STOP");
                if (i2cmotor<4)
                    {
                    twi_state=0;
                    i2c_start();
                  }
                    else
                    {
                    motor=0;    
                    }
                //I2CTimeout = 10;
                //i2c_start();   
                break; 
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Reading Data
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        /*
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
	*/
        default: 
                twi_state = 0;
                break;        
		}
 TWCR |= 0x80;
}


void I2cMotorWrite()
{
   twi_state = 0; // Azzero stato di comunicazione con i motori
   i2cmotor=0; 
   i2c_start(); // Do' il via all'informazione verso il motore dopo che ho ricevuto info x tutti e 4.
   
}

/* ------------------------------ Vecchio software -----------------------------------------------------------*/
void configureMotors() {
  #ifdef ServoControl
    frontMotor.attach(FRONTMOTORPIN);
    rearMotor.attach(REARMOTORPIN);
    rightMotor.attach(RIGHTMOTORPIN);
    leftMotor.attach(LEFTMOTORPIN);
  #endif
  
  #ifdef AnalogWrite
 //   analogWrite(FRONTMOTORPIN, 124);		
 //   analogWrite(REARMOTORPIN, 124);		
 //   analogWrite(RIGHTMOTORPIN, 124);		
 //   analogWrite(LEFTMOTORPIN, 124);		
  #endif
  
  #ifdef I2CWrite
    unsigned int timer,i,timer2 = 0;
//    DDRB  = 0x00;
//    PORTB = 0x00;
/*
    for(timer = 0; timer < 1000; timer++); 
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
    // In realta' questo e' l'init della scheda
*/
//   DDRC  = 0x81; // SCL
//    DDRC  |=0x40; // HEF4017 Reset
 //   PORTC = 0xff; // Pullup SDA
  //  DDRB  = 0x1B; // LEDs und Druckoffset
  //  PORTB = 0x01; // LED_Rot
  //  DDRD  = 0x3E; // Speaker & TXD & J3 J4 J5
  //  PORTD = 0x47; // LED

//  MCUSR &=~(1<<WDRF);  
//   WDTCSR |= (1<<WDCE)|(1<<WDE);
//  WDTCSR = 0;
//    PORTD=0;

  //Serial.print("Configurazioni motori I2C");
  //i2c_reset();
  i2c_init();
  //sei();
  i2cmotor=0; 
  //Serial.print("Spengo i motori");
/*
  for(mnumber=1;mnumber<7;mnumber++)
  {
    I2cMotorWrite(mnumber,0);
    Motor[mnumber]=0;
    delay(20);
  }
*/  
  #endif

}

void commandMotors() {
  #ifdef ServoControl
    //frontMotor.write(motorCommand[FRONT]);
    //rearMotor.write(motorCommand[REAR]);
    //rightMotor.write(motorCommand[RIGHT]);
    //leftMotor.write(motorCommand[LEFT]);
  #endif
  #ifdef AnalogWrite		
    //analogWrite(FRONTMOTORPIN, (motorCommand[FRONT] * mMotorCommand) + bMotorCommand);		
    //analogWrite(REARMOTORPIN, (motorCommand[REAR] * mMotorCommand) + bMotorCommand);		
    //analogWrite(RIGHTMOTORPIN, (motorCommand[RIGHT] * mMotorCommand) + bMotorCommand);		
    //analogWrite(LEFTMOTORPIN, (motorCommand[LEFT] * mMotorCommand) + bMotorCommand);		
  #endif
  
  #ifdef I2CWrite 
    Motor[FRONTMOTORID]=(motorCommand[FRONT] * mMotorCommand) + bMotorCommand;  // Setto la potenza in uscita per il motore selezionato
    Motor[REARMOTORID]=(motorCommand[REAR] * mMotorCommand) + bMotorCommand;  // Setto la potenza in uscita per il motore selezionato
    Motor[RIGHTMOTORID]=(motorCommand[RIGHT] * mMotorCommand) + bMotorCommand;  // Setto la potenza in uscita per il motore selezionato
    Motor[LEFTMOTORID]=(motorCommand[LEFT] * mMotorCommand) + bMotorCommand;  // Setto la potenza in uscita per il motore selezionato
    I2cMotorWrite();	
    /*
    Serial.print((motorCommand[FRONT] * mMotorCommand) + bMotorCommand);
    Serial.print(":");
    Serial.print((motorCommand[REAR] * mMotorCommand) + bMotorCommand);
    Serial.print(":");
    Serial.print((motorCommand[RIGHT] * mMotorCommand) + bMotorCommand);
    Serial.print(":");
    Serial.print((motorCommand[LEFT] * mMotorCommand) + bMotorCommand);
    Serial.println();
    */
    #endif  
}

// Sends commands to all motors
void commandAllMotors(int motorCommand) {
  #ifdef ServoControl
    frontMotor.write(motorCommand);
    rearMotor.write(motorCommand);
    rightMotor.write(motorCommand);
    leftMotor.write(motorCommand);
  #endif
 
  #ifdef AnalogWrite		
    //analogWrite(FRONTMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);		
    //analogWrite(REARMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);		
    //analogWrite(RIGHTMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);		
    //analogWrite(LEFTMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);		
  #endif

  #ifdef I2CWrite 
    Motor[FRONTMOTORID]=(motorCommand * mMotorCommand) + bMotorCommand;  // Setto la potenza in uscita per il motore selezionato
    Motor[REARMOTORID]=(motorCommand * mMotorCommand) + bMotorCommand;  // Setto la potenza in uscita per il motore selezionato
    Motor[RIGHTMOTORID]=(motorCommand * mMotorCommand) + bMotorCommand;  // Setto la potenza in uscita per il motore selezionato
    Motor[LEFTMOTORID]=(motorCommand * mMotorCommand) + bMotorCommand;  // Setto la potenza in uscita per il motore selezionato
    I2cMotorWrite();	
  #endif

}

void pulseMotors(byte quantity) {
  for (byte i = 0; i < quantity; i++) {      
    commandAllMotors(MINCOMMAND + 100);
    delay(250);
    commandAllMotors(MINCOMMAND);
    delay(250);
  }
}
