/*
AD4116_Setup.h
Definitions for setup configuration register bits for AD4116.
Part of Adopticum_AD411x Analog to digital converter Arduino library

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#pragma once
#include <Arduino.h>

namespace AD4116
{
	// The setup configuration registers are 16-bit registers 
	// that configure the reference selection, input buffers 
	// and output coding of the ADC. 
	// The layout for SETUPCON0 to SETUPCON7 is identical.
	namespace Setup
	{
		// bits 15..13 are reserved. Set to 0!

		// bit 12 Bipolar and Unipolar output coding.
		// 0 = unipolar coded output, 1 = bipolar coded output.
		const uint16_t UNIPOLAR = 0x0000;
		const uint16_t BIPOLAR = 0x1000;

		// bit 11 REFBUF+ This bit enables or disables the REF+ input buffer.
		const uint16_t REFBUF_P = 0x0800;
	
		// bit 10 REFBUF- This bit enables or disables the REF- input buffer.
		const uint16_t REFBUF_N = 0x0400;

		// bits 9..8 INBUF enables or disables input buffers.
		// 00 = Disable input buffers.
		// 01 = Reserved.
		// 10 = Reserved.
		// 11 = Enable input buffers.
		const uint16_t INPUT_BUFFERS = 0x0300;

		// bits 7..6 are reserved. Set to 0!

		// bits	5..4 REF_SEL selects the reference source for ADC conversion.
		// 00 = External reference (REF+-).
		// 10 = Internal reference (2.5V). Must also be enabled in via ADCMODE.
		// 11 = AVDD - AVSS. Low voltage reference.
		const uint16_t EXTERNAL_REF = 0x0000;
		const uint16_t INTERNAL_REF = 0x0020;
		const uint16_t AVDD_AVSS_REF = 0x0030;

		// bits 3..0 are reserved. Set to 0!
		const uint16_t RESERVED = 0x0000;
	};
}
