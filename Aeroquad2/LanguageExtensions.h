#ifndef __LANGUAGEEXTENSIONS_H__
#define __LANGUAGEEXTENSIONS_H__

#include <stdlib.h>

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

#endif