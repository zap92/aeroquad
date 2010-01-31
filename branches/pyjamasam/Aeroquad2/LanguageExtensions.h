#ifndef __LANGUAGEEXTENSIONS_H__
#define __LANGUAGEEXTENSIONS_H__

#include <stdlib.h>

//void * operator new(size_t size);
//void operator delete(void * ptr);

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

#endif