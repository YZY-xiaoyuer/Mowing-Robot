#include "crc.h"


//--------------------------------------------

//--------------------------------------------

//--------------------------------------------

//--------------------------------------------

/*
CRC校验类型：CRC8/MAXIM
多项式：X8+X5+X4+1
Poly：0011 0001  0x31
高位放到后面就变成 1000 1100 0x8c
*/
unsigned char crc8_chk_value(unsigned char *message, unsigned char len)
{
     uint8_t crc = 0;
     uint8_t i;
    while(len--)
      {
        crc ^= *message++;
      for(i = 0;i < 8;i++)
           {
           if(crc & 0x01)
             {
              crc = (crc >> 1) ^ 0x8c;
             }
             else crc >>= 1;
           }
      }
     return crc; 
}
//--------------------------------------------

