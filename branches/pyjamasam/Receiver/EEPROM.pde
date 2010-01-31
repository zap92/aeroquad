
void EEPROM_writeInt(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

unsigned int EEPROM_readInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

float EEPROM_readFloat(int p_address) 
{
  union floatStore 
  {
    byte floatByte[4];
    float floatVal;
  } 
  floatOut;

  for (int i = 0; i < 4; i++) 
  {
    floatOut.floatByte[i] = EEPROM.read(p_address + i);
  }
  return floatOut.floatVal;
}

void EEPROM_writeFloat(int p_address, float p_value) 
{
  union floatStore 
  {
    byte floatByte[4];
    float floatVal;
  } 
  floatIn;

  floatIn.floatVal = p_value;
  for (int i = 0; i < 4; i++) 
  {
    EEPROM.write(p_address + i, floatIn.floatByte[i]);
  }
}



