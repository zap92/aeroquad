#ifndef __LANGUAGEEXTENSIONS_H__
#define __LANGUAGEEXTENSIONS_H__

#include <stdlib.h>

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

void * operator new(size_t size);
void operator delete(void * ptr);

float fmap(float value, float istart, float istop, float ostart, float ostop);
float fconstrain(float data, float minLimit, float maxLimit);

int findMode(int *data, int arraySize);

const int freeMemory();

void normalize3DVector(float* vector);

#define Bit_Set(p,m) ((p) |= (1<<m)) 
#define Bit_Clear(p,m) ((p) &= ~(1<<m))

void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3]);
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2);
float Vector_Dot_Product(float vector1[3],float vector2[3]);
void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3]);
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3]);

#endif //__LANGUAGEEXTENSIONS_H__

