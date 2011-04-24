///*
//  AeroQuad v3.0 - April 2011
//  www.AeroQuad.com
//  Copyright (c) 2011 Ted Carancho.  All rights reserved.
//  An Open Source Arduino based multicopter.
// 
//  This program is free software: you can redistribute it and/or modify 
//  it under the terms of the GNU General Public License as published by 
//  the Free Software Foundation, either version 3 of the License, or 
//  (at your option) any later version. 
//
//  This program is distributed in the hope that it will be useful, 
//  but WITHOUT ANY WARRANTY; without even the implied warranty of 
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
//  GNU General Public License for more details. 
//
//  You should have received a copy of the GNU General Public License 
//  along with this program. If not, see <http://www.gnu.org/licenses/>. 
//*/
//





///******************************************************/
///********* AeroQuad Mini v1 Accelerometer *************/
///******************************************************/
//#if defined(AeroQuad_Mini)
//class Accel_AeroQuadMini : public Accel {
//private:
//  int accelAddress;
//  
//public:
//  Accel_AeroQuadMini() : Accel(){
//    accelAddress = 0x53; // page 10 of datasheet
//    accelScaleFactor = G_2_MPS2(4.0/1024.0);  // +/- 2G at 10bits of ADC
//  }
//  
//  void initialize(void) {
//    byte data;
//    
//    this->_initialize(0,1,2);  // AKA added for consistency
//  
//    accelOneG        = readFloat(ACCEL1G_ADR);
//    accelZero[XAXIS] = readFloat(LEVELPITCHCAL_ADR);
//    accelZero[YAXIS] = readFloat(LEVELROLLCAL_ADR);
//    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
//    smoothFactor     = readFloat(ACCSMOOTH_ADR);
//    
//    // Check if accel is connected
//    
//    if (readWhoI2C(accelAddress) !=  0xE5) // page 14 of datasheet
//      Serial.println("Accelerometer not found!");
//
//    updateRegisterI2C(accelAddress, 0x2D, 1<<3); // set device to *measure*
//    updateRegisterI2C(accelAddress, 0x31, 0x08); // set full range and +/- 2G
//    updateRegisterI2C(accelAddress, 0x2C, 8+2+1);    // 200hz sampling
//    delay(10); 
//  }
//  
//  void measure(void) {
//
//    sendByteI2C(accelAddress, 0x32);
//    Wire.requestFrom(accelAddress, 6);
//    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
//      if (axis == XAXIS)
//        accelADC[axis] = ((Wire.receive()|(Wire.receive() << 8))) - accelZero[axis];
//      else
//        accelADC[axis] = accelZero[axis] - ((Wire.receive()|(Wire.receive() << 8)));
//      //accelData[axis] = computeFirstOrder(accelADC[axis] * accelScaleFactor, &firstOrder[axis]);
//      accelData[axis] = filterSmooth(accelADC[axis] * accelScaleFactor, accelData[axis], smoothFactor);
//    }
//  }
//
//  const int getFlightData(byte axis) {
//      return getRaw(axis);
//  }
//  
//  // Allows user to zero accelerometers on command
//  void calibrate(void) {  
//    int findZero[FINDZERO];
//    int dataAddress;
//    
//    for (byte calAxis = XAXIS; calAxis < ZAXIS; calAxis++) {
//      if (calAxis == XAXIS) dataAddress = 0x32;
//      if (calAxis == YAXIS) dataAddress = 0x34;
//      if (calAxis == ZAXIS) dataAddress = 0x36;
//      for (int i=0; i<FINDZERO; i++) {
//        sendByteI2C(accelAddress, dataAddress);
//        findZero[i] = readReverseWordI2C(accelAddress);
//        delay(10);
//      }
//      accelZero[calAxis] = findMedianInt(findZero, FINDZERO);
//    }
//
//    // replace with estimated Z axis 0g value
//    accelZero[ZAXIS] = (accelZero[XAXIS] + accelZero[PITCH]) / 2;
//    // store accel value that represents 1g
//    measure();
//    accelOneG = -accelData[ZAXIS];
//     
//    writeFloat(accelOneG,        ACCEL1G_ADR);
//    writeFloat(accelZero[XAXIS], LEVELPITCHCAL_ADR);
//    writeFloat(accelZero[YAXIS], LEVELROLLCAL_ADR);
//    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
//  }
//};
//#endif



///******************************************************/
///****************** Wii Accelerometer *****************/
///******************************************************/
//#if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
//#include <Platform_Wii.h>  // @todo: Kenny, remove this when accel will be extracted as libraries!
//class Accel_Wii : public Accel {
//private:
//  Platform_Wii *platformWii;
//public:
//  Accel_Wii() : Accel(){
//    accelScaleFactor = 0.09165093;  // Experimentally derived to produce meters/s^2    
//  }
//  
//  void setPlatformWii(Platform_Wii *platformWii) {
//    this->platformWii = platformWii;
//  }
//  
//  void initialize(void) {
//    accelOneG        = readFloat(ACCEL1G_ADR);
//    accelZero[XAXIS] = readFloat(LEVELPITCHCAL_ADR);
//    accelZero[YAXIS] = readFloat(LEVELROLLCAL_ADR);
//    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
//    smoothFactor     = readFloat(ACCSMOOTH_ADR);
//  }
//  
//  void measure(void) {
//    // Actual measurement performed in gyro class
//    // We just update the appropriate variables here
//    // Depending on how your accel is mounted, you can change X/Y axis to pitch/roll mapping here
//    platformWii->measure();
//    accelADC[XAXIS] =  platformWii->getAccelADC(PITCH) - accelZero[PITCH];
//    accelADC[YAXIS] = platformWii->getAccelADC(ROLL) - accelZero[ROLL];
//    accelADC[ZAXIS] = accelZero[ZAXIS] - platformWii->getAccelADC(ZAXIS);
//    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
//      accelData[axis] = filterSmooth(accelADC[axis] * accelScaleFactor, accelData[axis], smoothFactor);
//    }
//  }
//  
//  const int getFlightData(byte axis) {
//      return getRaw(axis);
//  }
// 
//  // Allows user to zero accelerometers on command
//  void calibrate(void) {
//    int findZero[FINDZERO];
//
//    for(byte calAxis = XAXIS; calAxis < LASTAXIS; calAxis++) {
//      for (int i=0; i<FINDZERO; i++) {
//        platformWii->measure();
//        findZero[i] = platformWii->getAccelADC(calAxis);
//      }
//      accelZero[calAxis] = findMedianInt(findZero, FINDZERO);
//    }
//    
//    // store accel value that represents 1g
//    accelOneG = -accelData[ZAXIS];
//    // replace with estimated Z axis 0g value
//    accelZero[ZAXIS] = (accelZero[XAXIS] + accelZero[YAXIS]) / 2;
//    
//    writeFloat(accelOneG, ACCEL1G_ADR);
//    writeFloat(accelZero[YAXIS], LEVELROLLCAL_ADR);
//    writeFloat(accelZero[XAXIS], LEVELPITCHCAL_ADR);
//    writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
//  }
//};
//#endif


///******************************************************/
///****************** CHR6DM Accelerometer **************/
///******************************************************/
//#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
//class Accel_CHR6DM : public Accel {
//public:
//  Accel_CHR6DM() : Accel() {
//    accelScaleFactor = 0;
//  }
//
//  void initialize(void) {
////    smoothFactor = readFloat(ACCSMOOTH_ADR);
////    accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
////    accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
////    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
////    accelOneG = readFloat(ACCEL1G_ADR);
////    calibrate();
//
//    // do nothing, but still be compatible with the rest of the classes, all responsibility is in gyro class for CHR6DM
//  }
//
//  void measure(void) {
//      //accelADC[XAXIS] = chr6dm.data.ax - accelZero[XAXIS];
//      //accelADC[YAXIS] = chr6dm.data.ay - accelZero[YAXIS];
//      //accelADC[ZAXIS] = chr6dm.data.az - accelOneG;
//
//      //accelData[XAXIS] = filterSmooth(accelADC[XAXIS], accelData[XAXIS], smoothFactor); //to get around 1
//      //accelData[YAXIS] = filterSmooth(accelADC[YAXIS], accelData[YAXIS], smoothFactor);
//      //accelData[ZAXIS] = filterSmooth(accelADC[ZAXIS], accelData[ZAXIS], smoothFactor);
//      
//    // do nothing, but still be compatible with the rest of the classes, all responsibility is in gyro class for CHR6DM
//  }    
//
//  const int getFlightData(byte axis) {
//    return getRaw(axis);
//  }
//
//  // Allows user to zero accelerometers on command
//  void calibrate(void) {
//    // do nothing
//  }
//};
//#endif
//

