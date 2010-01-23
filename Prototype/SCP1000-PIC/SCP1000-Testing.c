/*
    10-15-06
    Nathan Seidle
    nathan@sparkfun.com
    Copyright Spark Fun Electronics© 2006
    
    Software SPI control
    PIC 16F88
    
    Reading and writing to SCP1000-D01 Barometric pressure sensor
    
    This is a very precise (19-bit!!), configurable pressure sensor
    
    Be sure to read and re-read the datasheet. Once it's fully absorbed,
    the interface is pretty straight forward.
    
    Choose your processor and compiler well. This sensor uses 16 and 19-bit decimals
    It is tricky to get the data into a contained, printable value on an 8-bit PIC.
    We decided to print hex values for the pressure data and then convert
    hex to decimal on the computer using window's calculator.
    
*/
#define Clock_8MHz
#define Baud_9600

#include "c:\Global\Code\C\16F88.h"  // device dependent interrupt definitions

#pragma origin 4

#include "c:\Global\Code\Pics\Code\Delay.c"   //Standard delays
#include "c:\Global\Code\Pics\Code\stdio.c"   //Software based Basic Serial IO

#include "c:\Global\Code\C\math24f.h"   //

#define STATUS_LED PORTB.3

#define GND  PORTA.0 //GND
#define PWR  PORTA.1 //3.3V
#define SCL  PORTA.2 //SCK This is clock input into SCP1000
#define SDI  PORTB.0 //MOSI This is input into the SCP1000 from the PIC
#define SDO  PORTA.3 //MISO This is output from SCP1000 into the PIC
#define CS   PORTA.4 //CSB This is chip select input - when low, the SCP1000 is selected
#define DRDY PORTB.7 //DRDY This is Data ready interrupt output from SCP1000


void boot_up(void);

uns8 spi_comm(uns8 outgoing_byte);
uns8 read_register(uns8 register_name);
uns16 read_register16(uns8 register_name);
void write_register(uns8 register_name, uns8 register_value);
void SCP1000_Test(void);


void main(void)
{

    uns8 choice;
    
    boot_up();

    SCP1000_Test();
    
    while(1);

}//End Main

void boot_up(void)
{
    OSCCON = 0b.0111.0000; //Setup internal oscillator for 8MHz
    while(OSCCON.2 == 0); //Wait for frequency to stabilize

    //Setup Ports
    ANSEL = 0b.0000.0000; //Turn off A/D

    //Read RA3
    PORTA = 0b.0000.0010;
    TRISA = 0b.0000.1000;  //0 = Output, 1 = Input

    //Input RB7 (DRDY), Output RB0
    PORTB = 0b.0000.0000;
    TRISB = 0b.1000.0100;   //0 = Output, 1 = Input RX on RB2

    //Setup the hardware UART module
    //=============================================================
    SPBRG = 51; //8MHz for 9600 inital communication baud rate
    //SPBRG = 129; //20MHz for 9600 inital communication baud rate

    TXSTA = 0b.0010.0100; //8-bit asych mode, high speed uart enabled
    RCSTA = 0b.1001.0000; //Serial port enable, 8-bit asych continous receive mode
    //=============================================================

    PWR = 1; //Power the unit with 3.3V
    GND = 0; 
    
    CS = 1; //Unselect SCP
    
}

//Read data from SCP1000 via SPI
void SCP1000_Test(void)
{

    uns8 out_byte, in_byte, choice;
    uns16 big_in_byte;

    PWR = 1; //Power the unit with 3.3V
    GND = 0; 
    
    CS = 1; //Unselect SCP

    STATUS_LED = 0;

    //Configure SCP1000 with low noise configuration
    //=====================================
    //write_register(place, thing)
    write_register(0x02, 0x2D);
    write_register(0x01, 0x03);
    write_register(0x03, 0x02);
    
    delay_ms(100);
    //=====================================

    while(1)
    {
        if(RCIF)
        {
            choice = RCREG;
            
            if(choice == 'x') break;
        }

        STATUS_LED = 1;

        //Select High Resolution Mode
        write_register(0x03, 0x0A);
        //Acquisistion starts
        while(DRDY == 0); //Wait for data ready pin to go high

        STATUS_LED = 0;
        
        uns16 temp_data;
        temp_data = read_register16(0x21); //Read the temperature data
        temp_data /= 2; //This is (temp * 10) / 20 so that the answer of 27.9 comes out as 279 (easier to print)
        printf("Temp=%d ", temp_data);
    
        uns8 pressure_data_high;
        uns16 pressure_data_low;
    
        pressure_data_high = read_register(0x1F); //Read the pressure data MSB 3-bits
        pressure_data_high &= 0b.0000.0111; //Data bits are 2:0
    
        pressure_data_low = read_register16(0x20); //Read the pressure data LSB 16-bits
        
        printf("Pressure[Pa]=%h-", pressure_data_high);
        printf("%h ", pressure_data_low);
            
        in_byte = read_register(0x01); //Read ASIC revision
        printf("asic_rev=%h ", in_byte);
    
        in_byte = read_register(0x07); //Read STATUS
        printf("STATUS=%b\n", in_byte);

    }

}

//Read 8-bit register
uns8 read_register(uns8 register_name)
{
    uns8 in_byte;
    
    register_name <<= 2;
    register_name &= 0b.1111.1100; //Read command

    CS = 0; //Select SPI Device

    in_byte = spi_comm(register_name); //Write byte to device
    //in_byte is nothing, we need to clock in another 8 bits
    in_byte = spi_comm(0x00); //Send nothing, but we should get back the register value

    CS = 1;
    
    return(in_byte);
}

//Read 16-bit register
uns16 read_register16(uns8 register_name)
{
    uns16 in_byte;
    
    register_name <<= 2;
    register_name &= 0b.1111.1100; //Read command

    CS = 0; //Select SPI Device

    in_byte.high8 = spi_comm(register_name); //Write byte to device
    //in_byte is nothing, we need to clock in another 8 bits
    in_byte.high8 = spi_comm(0x00); //Send nothing, but we should get back the register value
    in_byte.low8 = spi_comm(0x00); //Send nothing, but we should get back the register value

    CS = 1;
    
    return(in_byte);
}

//Sends a write command to SCP1000
void write_register(uns8 register_name, uns8 register_value)
{
    uns8 in_byte;
    
    register_name <<= 2;
    register_name |= 0b.0000.0010; //Write command

    CS = 0; //Select SPI device

    in_byte = spi_comm(register_name); //Send register location
    in_byte = spi_comm(register_value); //Send value to record into register
    
    CS = 1;
    
    //Return nothing
}

//Basic SPI send and receive
uns8 spi_comm(uns8 outgoing_byte)
{
    uns8 incoming_byte, x;

    for(x = 0 ; x < 8 ; x++)
    {
        SCL = 0; //Toggle the SPI clock

        SDI = outgoing_byte.7; //Put bit on SPI data bus
        outgoing_byte <<= 1; //Rotate byte 1 to the left

        SCL = 1;

        incoming_byte <<= 1; //Rotate byte 1 to the left
        incoming_byte.0 = SDO; //Read bit on SPI data bus
    }
    
    return(incoming_byte);
}