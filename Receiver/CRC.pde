unsigned char KIRSP_CRC16_Hi_Byte  = 0xFF;  // Do not modify
unsigned char KIRSP_CRC16_Low_Byte = 0xFF;  // Do not modify

unsigned int KIRSP_cal_CRC16(unsigned char *CRC16_Data_Array, unsigned short CRC16_Data_Array_Len)
{
  unsigned int x = 0xFFFF;
  unsigned int y;
  int i;
  x ^= *CRC16_Data_Array;
  for (i = 0; i < 8; ++i)
  {
    if (x & 1)
      x = (x >> 1) ^ 0xA001; // <----The key
    else
      x = (x >> 1);
  }
  KIRSP_CRC16_Hi_Byte = highByte(x);
  KIRSP_CRC16_Low_Byte = lowByte(x);
  y = x;  
  CRC16_Data_Array_Len--;
  *CRC16_Data_Array++;  
  while (CRC16_Data_Array_Len--)
  {
    y ^= *CRC16_Data_Array;
    for (i = 0; i < 8; ++i)
    {
      if (y & 1)
        y = (y >> 1) ^ 0xA001; // <---The Key
      else
        y = (y >> 1);
    }
    KIRSP_CRC16_Hi_Byte = highByte(y);
    KIRSP_CRC16_Low_Byte = lowByte(y);  
    *CRC16_Data_Array++;
  }  
  KIRSP_CRC16_Hi_Byte = lowByte(y);   // write to global variable
  KIRSP_CRC16_Low_Byte = highByte(y); // write to global variable
  return y;
} // end of KIRSP_cal_CRC16 function

