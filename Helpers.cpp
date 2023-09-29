#include <Arduino.h>

namespace Helpers
{
  // Some generic helper functions. ---------------------------------------------

    byte set_bit(byte data, byte bit, bool value)
    {
      byte mask = 0x01 << bit;
      bit = value ? 0xff : 0x00;
      data = (data & ~mask) | (bit & mask);
      return data;
    }

    void print_bytes(byte *data, byte data_len) 
    {
      char sz[4];
      for (auto i=0; i<data_len; i++) { 
        snprintf(sz, 4, " %02X", data[i]);
        Serial.print(sz); 
      }
    }
};
