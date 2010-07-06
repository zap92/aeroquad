/To read a value
char ITG3200Read(unsigned char address)
{
  char data=0x00;
      
     Wire.beginTransmission(0x64);
     Wire.send((address));  
         delay(5);
         Wire.endTransmission();
     Wire.requestFrom(0x64,1);
     if (Wire.available()>0)
         {
           data = Wire.receive();
       return data;
         }
        Wire.endTransmission();
}



void Gyro_Init()
{
 Wire.beginTransmission(0x64);
 Wire.send(0x3E);
 Wire.send(0x80);  //send a reset to the device
 Wire.endTransmission(); //end transmission

 Wire.beginTransmission(0x64);
 Wire.send(0x15);
 Wire.send(0x00);   //sample rate divider
 Wire.endTransmission(); //end transmission

 Wire.beginTransmission(0x64);
 Wire.send(0x16);
 Wire.send(0x18); // ±2000 degrees/s (default value)
 Wire.endTransmission(); //end transmission

 Wire.beginTransmission(0x64);
 Wire.send(0x17);
 Wire.send(0x05);   // enable send raw values
 Wire.endTransmission(); //end transmission

//  Wire.beginTransmission(0x64);
//  Wire.send(0x3E);
//  Wire.send(0x00);  
//  Wire.endTransmission(); //end transmission
}
