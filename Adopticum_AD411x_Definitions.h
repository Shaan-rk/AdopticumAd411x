/*
Adopticum_AD411x
AD411x Analog to digital converter Arduino library

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef __Adopticum_AD411x_Definitions_h
#define __Adopticum_AD411x_Definitions_h

//TODO: namespace AD411x

namespace Registers
{
    enum {
      COMMS = 0x00,
      STATUS = 0x00,
      ADCMOD = 0x01,
      IFMODE = 0x02,
      REGCHECK = 0x03,
      DATA = 0x04,
      GPIOCON = 0x06,
      ID = 0x07,
      // ADC channel registers.
      CH0 = 0x10,
      CH1 = 0x11,
      CH2 = 0x12,
      CH3 = 0x13,
      CH4 = 0x14,
      CH5 = 0x15,
      CH6 = 0x16,
      CH7 = 0x17,
      CH8 = 0x18,
      CH9 = 0x19,
      CH10 = 0x1A,
      CH11 = 0x1B,
      CH12 = 0x1C,
      CH13 = 0x1D,
      CH14 = 0x1E,
      CH15 = 0x1F,
      // ADC setup config registers.
      SETUP0 = 0x20,
      SETUP1 = 0x21,
      SETUP2 = 0x22,
      SETUP3 = 0x23,
      SETUP4 = 0x24,
      SETUP5 = 0x25,
      SETUP6 = 0x26,
      SETUP7 = 0x27,
      // ADC filter config registers.
      FILTER0 = 0x28,
      FILTER1 = 0x29,
      FILTER2 = 0x2A,
      FILTER3 = 0x2B,
      FILTER4 = 0x2C,
      FILTER5 = 0x2D,
      FILTER6 = 0x2E,
      FILTER7 = 0x2F,
      // ADC offset registers.
      OFFSET0 = 0x30,
      OFFSET1 = 0x31,
      OFFSET2 = 0x32,
      OFFSET3 = 0x33,
      OFFSET4 = 0x34,
      OFFSET5 = 0x35,
      OFFSET6 = 0x36,
      OFFSET7 = 0x37,
      // ADC gain registers.
      GAIN0 = 0x38,
      GAIN1 = 0x39,
      GAIN2 = 0x3A,
      GAIN3 = 0x3B,
      GAIN4 = 0x3C,
      GAIN5 = 0x3D,
      GAIN6 = 0x3E,
      GAIN7 = 0x3F
    };
}

namespace Datarate
{
  // ADC filter data rates (samples per second) some are are rounded down.
  // The data rates are for sinc5 + sinc1.
  enum {
    SPS_31250 = 0x00,
    SPS_15625 = 0x06,
    SPS_10417 = 0x07,
    SPS_5208 = 0x08,
    SPS_2597 = 0x09,
    SPS_1007 = 0x0A,
    SPS_503 = 0x0B,
    SPS_381 = 0x0C,
    SPS_200 = 0x0D,
    SPS_100 = 0x0E,
    SPS_59 = 0x0F,
    SPS_49 = 0x10,
    SPS_20 = 0x11,
    SPS_16 = 0x12,
    SPS_10 = 0x13,
    SPS_5 = 0x14,
    SPS_2 = 0x15,
    SPS_1 = 0x16
  };
}

namespace OutputDataRate
{
// These bits control the output data rate of the ADC and, therefore, the settling time
// and noise for Setup x. Rates shown are for single channel enabled sinc5 + sinc 1 filter.
// See Table 7 for multiple channels enabled.
  enum {
    SPS_62500_0 = 0b00000,  // 62500 SPS.
    SPS_62500_1 = 0b00001,  // 62500 SPS.
    SPS_62500_2 = 0b00010,  // 62500 SPS.
    SPS_62500_3 = 0b00011,  // 62500 SPS.
    SPS_31250_0 = 0b00100,  // 31250 SPS.
    SPS_31250_1 = 0b00101,  // 31250 SPS.
    SPS_15625   = 0b00110,  // 15625 SPS.
    SPS_10416   = 0b00111,  // 10416.7 SPS.
    SPS_5194    = 0b01000,  // 5194.8 SPS (5208.3 SPS for sinc3).
    SPS_2496    = 0b01001,  // 2496.9 SPS (2500 SPS for sinc3).
    SPS_1007    = 0b01010,  // 1007.6 SPS (1008.1 SPS for sinc3).
    SPS_500     = 0b01011,  // 499.9 SPS (500 SPS for sinc3).
    SPS_390     = 0b01100,  // 390.6 SPS (400.64 SPS for sinc3).
    SPS_200     = 0b01101,  // 200.3 SPS (200.32 SPS for sinc3).
    SPS_100     = 0b01110,  // 100.0 SPS.
    SPS_60      = 0b01111,  // 59.75 SPS (59.98 SPS for sinc3).
    SPS_50      = 0b10000,  // 49.84 SPS (50 SPS for sinc3).
    SPS_20      = 0b10001,  // 20.00 SPS.
    SPS_16      = 0b10010,  // 16.65 SPS (16.67 SPS for sinc3).
    SPS_10      = 0b10011,  // 10.00 SPS.
    SPS_5       = 0b10100,  // 5.00 SPS.
    SPS_2       = 0b10101,  // 2.50 SPS.
    SPS_1       = 0b10110   // 1.25 SPS.
  };
}

namespace CodingMode
{ // ADC setup coding modes.
  enum {
    UNIPOLAR = 0x00,
    BIPOLAR = 0x01
  };
}

namespace DataMode
{ // ADC data conversion modes.
  enum {
    CONTINUOUS_CONVERSION_MODE = 0x00,
    SINGLE_CONVERSION_MODE = 0x01,
    CONTINUOUS_READ_MODE
  };
}

namespace ClockMode
{
  enum {
    //	00 Internal oscillator
    INTERNAL_CLOCK = 0x00,
    //	01 Internal oscillator output on XTAL2/CLKIO pin
    INTERNAL_CLOCK_OUTPUT = 0x01,
    //	10 External clock input on XTAL2/CLKIO pin
    EXTERNAL_CLOCK_INPUT = 0x02,
    //	11 External crystal on XTAL1 and XTAL2/CLKIO pins
    EXTERNAL_CRYSTAL = 0x03
  };
}

namespace BufferMode
{ // ADC channel buffer setting.
  enum {
    AIN_BUF_DISABLE = 0x00,
    AIN_BUF_ENABLE = 0x03
  };
}

namespace ReferenceMode
{ // ADC internal reference modes.
  enum {
    REF_DISABLE = 0x00,
    REF_ENABLE = 0x01
  };
}

namespace ReferenceSource
{
  enum {
   // 00 External reference source
    REF_EXT = 0x00,
  	// 01 AIN1/REF2+ and AIN0/REF2−
    REF_AIN = 0x01,
  	// 10 Internal reference source
    REF_INT = 0x02,
  	// 11 External AVDD1 – AVSS
    REF_PWR = 0x03
  };
}

#endif