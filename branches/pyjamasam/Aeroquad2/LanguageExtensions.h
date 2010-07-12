#ifndef __LANGUAGEEXTENSIONS_H__
#define __LANGUAGEEXTENSIONS_H__

#include <stdlib.h>

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

void * operator new(size_t size)
{
  return malloc(size);
}

void operator delete(void * ptr)
{
  free(ptr);
}

float fmap(float value, float istart, float istop, float ostart, float ostop) 
{
	return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

float fconstrain(float data, float minLimit, float maxLimit) 
{
  if (data < minLimit) return minLimit;
  else if (data > maxLimit) return maxLimit;
  else return data;
}

int findMode(int *data, int arraySize) 
{
  	// The mode of a set of numbers is the value that occurs most frequently
	boolean done = 0;
	byte i;
	int temp, maxData, frequency, maxFrequency;

	// Sorts numbers from lowest to highest
	while (done != 1) 
	{        
		done = 1;
		for (i=0; i<(arraySize-1); i++) 
		{
			if (data[i] > data[i+1]) 
			{     // numbers are out of order - swap
				temp = data[i+1];
				data[i+1] = data[i];
				data[i] = temp;
				done = 0;
			}
		}
	}

	temp = 0;
	frequency = 0;
	maxFrequency = 0;

	// Count number of times a value occurs in sorted array
	for (i=0; i<arraySize; i++) 
	{
		if (data[i] > temp) 
		{
			frequency = 0;
			temp = data[i];
			frequency++;
		}
		else if (data[i] == temp) 
		{
			frequency++;
		}

		if (frequency > maxFrequency) 
		{
			maxFrequency = frequency;
			maxData = data[i];
		}
	}
	
	return maxData;
}


//Method to get the free memory on the arduino
//http://www.arduino.cc/playground/Code/AvailableMemory
uint8_t *heapptr, *stackptr;
const int freeMemory() 
{
  	stackptr = (uint8_t *)malloc(4);          // use stackptr temporarily
	heapptr = stackptr;                     // save value of heap pointer
	free(stackptr);      // free up the memory again (sets stackptr to 0)
	stackptr =  (uint8_t *)(SP);           // save value of stack pointer
	
	return stackptr - heapptr;
}

void normalize3DVector(float* vector)
{
	static float R;  
	R = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
	vector[0] /= R;
	vector[1] /= R;  
	vector[2] /= R;  
}



#define Bit_Set(p,m) ((p) |= (1<<m)) 
#define Bit_Clear(p,m) ((p) &= ~(1<<m))


//Computes the cross product of two vectors
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3])
{
  vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

//Multiply the vector by a scalar. 
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2)
{
  for(int c=0; c<3; c++)
  {
   vectorOut[c]=vectorIn[c]*scale2; 
  }
}


//Computes the dot product of two vectors
float Vector_Dot_Product(float vector1[3],float vector2[3])
{
  float op=0;
  
  for(int c=0; c<3; c++)
  {
  	op+=vector1[c]*vector2[c];
  }
  
  return op; 
}

void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3])
{
  for(int c=0; c<3; c++)
  {
     vectorOut[c]=vectorIn1[c]+vectorIn2[c];
  }
}

//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!). 
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3])
{
  float op[3]; 
  for(int x=0; x<3; x++)
  {
    for(int y=0; y<3; y++)
    {
      for(int w=0; w<3; w++)
      {
       op[w]=a[x][w]*b[w][y];
      } 
      mat[x][y]=0;
      mat[x][y]=op[0]+op[1]+op[2];
      
      float test=mat[x][y];
    }
  }
}
#endif