/*
AD4116_Inputs.h -
Definitions of input channel selection bits for AD4116.
Part of Adopticum_AD411x Analog to digital converter Arduino library

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#pragma once
#include <Arduino.h>

namespace AD4116
{
	// These bits select which input pair is connected to the input of the ADC for a channel.
	// These are bits [9..0] in the CHx registers, named INPUTx.
	enum class InputType : uint16_t
	{
		VIN0_VIN1 = 0b0000000001,	 // VIN0 and VIN1.
		VIN0_VINCOM = 0b0000010000,	 // VIN0 and VINCOM.
		VIN1_VIN0 = 0b0000100000,	 // VIN1 and VIN0.
		VIN1_VINCOM = 0b0000110000,	 // VIN1 and VINCOM.
		VIN2_VIN3 = 0b0001000011,	 // VIN2 and VIN3.
		VIN2_VINCOM = 0b0001010000,	 // VIN2 and VINCOM.
		VIN3_VIN2 = 0b0001100010,	 // VIN3 and VIN2.
		VIN3_VINCOM = 0b0001110000,	 // VIN3 and VINCOM.
		VIN4_VIN5 = 0b0010000101,	 // VIN4 and VIN5.
		VIN4_VINCOM = 0b0010010000,	 // VIN4 and VINCOM.
		VIN5_VIN4 = 0b0010100100,	 // VIN5 and VIN4.
		VIN5_VINCOM = 0b0010110000,	 // VIN5 and VINCOM.
		VIN6_VIN7 = 0b0011000111,	 // VIN6 and VIN7.
		VIN6_VINCOM = 0b0011010000,	 // VIN6 and VINCOM.
		VIN7_VIN6 = 0b0011100110,	 // VIN7 and VIN6.
		VIN7_VINCOM = 0b0011110000,	 // VIN7 and VINCOM.
		VIN8_VIN9 = 0b0100001001,	 // VIN8 and VIN9.
		VIN8_VINCOM = 0b0100010000,	 // VIN8 and VINCOM.
		VIN9_VIN8 = 0b0100101000,	 // VIN9 and VIN8.
		VIN9_VINCOM = 0b0100110000,	 // VIN9 and VINCOM.
		VIN10_VINCOM = 0b0101010000, // VIN10, VINCOM (single-ended or differential pair)

		ADCIN11_ADCIN12 = 0b0101101100, // ADCIN11, ADCIN12.
		ADCIN12_ADCIN11 = 0b0110001011, // ADCIN12, ADCIN11.
		ADCIN13_ADCIN14 = 0b0110101110, // ADCIN13, ADCIN14.
		ADCIN14_ADCIN13 = 0b0111001101, // ADCIN14, ADCIN13.
		ADCIN11_ADCIN15 = 0b0101101111, // ADCIN11, ADCIN15. (pseudo differential or differential pair)
		ADCIN12_ADCIN15 = 0b0110001111, // ADCIN12, ADCIN15. (pseudo differential or differential pair)
		ADCIN13_ADCIN15 = 0b0110101111, // ADCIN13, ADCIN15. (pseudo differential or differential pair)
		ADCIN14_ADCIN15 = 0b0111001111, // ADCIN14, ADCIN15. (pseudo differential or differential pair)
		Temperature = 0b1000110010,		// Temperature sensor.
		Reference = 0b1010110110		// Reference.
	};
}
