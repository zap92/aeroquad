#include "Spi.h"

#define LEDPIN 13
#define BARO_SS 47
#define BARO_DRDY 46
#define COMPASS_SS 53
#define COMPASS_DRDY 48
#define COMPASS_RESET 49

#define UBLB(a,b)  ( ( (a) << 8) | (b) )
#define UBLB19(a,b) ( ( (a) << 16 ) | (b) )

//Addresses
#define REVID 0x00	//ASIC Revision Number
#define OPSTATUS 0x04   //Operation Status
#define STATUS 0x07     //ASIC Status
#define START 0x0A      //Constant Readings
#define PRESSURE 0x1F   //Pressure 3 MSB
#define PRESSURE_LSB 0x20 //Pressure 16 LSB
#define TEMP 0x21       //16 bit temp

// Pin assignments for accelerometers
#define XACCPIN 0
#define YACCPIN 1
#define ZACCPIN 2

#define WAIT 3
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#include "WProgram.h"
int readAxis(byte axis);
void setup();
void loop();
char read_register(byte register_name);
float read_register16(char register_name);
void write_register(char register_name, char register_value);
int x = 0;        // magnetic field x axis
int y = 0;        // magnetic field y axis
int z = 0;        // magnetic field z axis
int xaccel = 0;
int yaccel = 0;
int zaccel = 0;
float heading = 0;  // magnetic field heading
float roll = 0;
float pitch = 0;
float g = 0;
float CMx = 0;
float CMy = 0;

char rev_in_byte;	    
int temp_in;
unsigned long pressureLSB;
unsigned long pressureMSB;
unsigned long temp_pressure;
unsigned long pressure;

int readAxis(byte axis){
  int measurement;
  
  // Send reset
  digitalWrite(COMPASS_RESET, HIGH);
  delayMicroseconds(WAIT);
  digitalWrite(COMPASS_RESET, LOW);

  // Send command byte
  // Description found on page 9 of MicroMag3 data sheet
  // First nibble defines speed/accuracy of measurement
  // Use 0x70 for the slowest/best accuracy, 0x10 for fastest/least accuracy
  // Last nibble defines axis (X = 0x01, Y = 0x02, Z - 0x03)
  switch (axis) {
    case XAXIS:
      Spi.transfer(0x61);
      break;
    case YAXIS:
      Spi.transfer(0x62);
      break;
    case ZAXIS:
      Spi.transfer(0x63);
      break;
  }

  // Wait for data to be ready, then read two bytes
  while(digitalRead(COMPASS_DRDY) == LOW);  
  return (Spi.transfer(0xFF) << 8) | Spi.transfer(0xFF);
}

void setup()
{
  Serial.begin(115200);
  pinMode(LEDPIN, OUTPUT); 
  digitalWrite(LEDPIN, HIGH);
  pinMode(COMPASS_DRDY, INPUT);
  pinMode(COMPASS_SS, OUTPUT);
  pinMode(COMPASS_RESET, OUTPUT);
  pinMode(BARO_DRDY, INPUT);
  pinMode(BARO_SS,OUTPUT);

  digitalWrite(COMPASS_SS, HIGH);
  digitalWrite(COMPASS_RESET, LOW);
  digitalWrite(BARO_SS,HIGH); //disable device  
  delay(1000);
  
  // Initialize barometer
  write_register(0x02, 0x2D);
  write_register(0x01, 0x03);
  write_register(0x03, 0x02);
  delay(1000);
  write_register(0x03,0x09);
}

void loop() {
  
  if (digitalRead(BARO_DRDY) == HIGH) {
    temp_in = read_register16(TEMP);
    temp_in = temp_in * 0.05;
    //temp_in = ((1.8) *temp_in) + 32;

    pressureMSB = read_register(PRESSURE);
    pressureMSB &= B00000111;
    pressureLSB = read_register16(PRESSURE_LSB);
    pressure = UBLB19(pressureMSB, pressureLSB);
    pressure = pressure * 0.25;
  }  
    Serial.print("Pressure [");
    Serial.print(pressure, DEC);
    Serial.print("]     ");
    Serial.print("Temperature C [");
    Serial.print(temp_in , DEC);
    Serial.print("]     ");

  
  //if (digitalRead(COMPASS_DRDY) == HIGH) {
    digitalWrite(COMPASS_SS, LOW);
    x = readAxis(XAXIS);  // read the x-axis magnetic field value
    y = readAxis(YAXIS);  // read the y-axis magnetic field value
    z = readAxis(ZAXIS);  // read the z-axis magnetic field value
    digitalWrite(COMPASS_SS, HIGH);

    // Estimated level position by rotating accels on each axis and found midpoint
    // Real implementation will use properly calibrated accelerometer values
    xaccel = 355-analogRead(XACCPIN);
    yaccel = 355-analogRead(YACCPIN);
    zaccel = analogRead(ZACCPIN)-355;
    /*Serial.print(xaccel);
    Serial.print(" ");
    Serial.print(yaccel);
    Serial.print(" ");
    Serial.println(zaccel);*/
  
    roll = atan2(xaccel, zaccel);
    pitch = atan2(yaccel, zaccel);
    /*Serial.print("roll = ");
    Serial.print(degrees(roll));
    Serial.print(", pitch = ");
    Serial.println(degrees(pitch));*/
  
    CMx = (x * cos(pitch)) + (y *sin(roll) * sin(pitch)) - (z * cos(roll) * sin(pitch));
    CMy = (y * cos(roll)) + (z * sin(roll));
    heading = abs(degrees(atan(CMy/CMx)));
    if (CMx >= 0 && CMy >= 0) {heading = 180 - heading;}
    if (CMx >= 0 && CMy < 0) {heading = heading + 180;}
    if (CMx < 0 && CMy < 0) {heading = 360 - heading;}
    /*Serial.print(xaccel);
    Serial.print(", ");
    Serial.print(yaccel);
    Serial.print(", ");
    Serial.print(zaccel);
    Serial.print(" | ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);*/
    Serial.print("Heading = ");
    Serial.println(heading);
  //}
}

char read_register(byte register_name) {
    char in_byte;
    register_name <<= 2;
    register_name &= B11111100; //Read command
  
    digitalWrite(BARO_SS,LOW); //Select SPI Device
    Spi.transfer(register_name); //Write byte to device
    in_byte = Spi.transfer(0x00); //Send nothing, but we should get back the register value
    digitalWrite(BARO_SS,HIGH);
    return(in_byte);
}

float read_register16(char register_name) {
    byte in_byte1;
    byte in_byte2;
    float in_word;
    
    register_name <<= 2;
    register_name &= B11111100; //Read command

    digitalWrite(BARO_SS,LOW); //Select SPI Device
    Spi.transfer(register_name); //Write byte to device
    in_byte1 = Spi.transfer(0x00);    
    in_byte2 = Spi.transfer(0x00);
    digitalWrite(BARO_SS,HIGH);
    in_word = UBLB(in_byte1,in_byte2);
    return(in_word);
}

void write_register(char register_name, char register_value) {
    register_name <<= 2;
    register_name |= B00000010; //Write command

    digitalWrite(BARO_SS,LOW); //Select SPI device
    Spi.transfer(register_name); //Send register location
    Spi.transfer(register_value); //Send value to record into register
    digitalWrite(BARO_SS,HIGH);
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

