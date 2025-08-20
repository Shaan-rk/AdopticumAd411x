/*
Adopticum_AD411x.h
Adopticum_AD411x Analog to Digital converter Arduino library.
To use the Adopticum_AD411x library, include this file in your Arduino sketch.

This library depends on the Adafruit BusIO library.
Make sure you also install the Adafruit BusIO library.

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#pragma once
#include "AD411x.h"
#include "AD4111.h"
#include "AD411x_Device.h"
#include "AdaptiveMovingAverage.h"
#include <Adafruit_SPIDevice.h>

// Fault bit definitions
#define AMPFAULT_NONE   0x00
#define AMPFAULT_OVUV   0x01  // Over/Under voltage (voltage channels)
#define AMPFAULT_RANGE  0x02  // Out-of-range (voltage channels)
#define AMPFAULT_OPEN   0x04  // Open-circuit (current channels)
