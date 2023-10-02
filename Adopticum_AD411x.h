/*
Adopticum_AD411x
AD411x Analog to digital converter Arduino library

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef __Adopticum_AD411x_h
#define __Adopticum_AD411x_h

#include <Adafruit_SPIDevice.h>
#include <Arduino.h>
#include <SPI.h>
#include "AD4116_Input.h"

class AD411x
{
  // Note: 
  // This implementation is limited to EXACTLY ONE AD411x CONNECTED TO THE MCU.
  // This class implements the singleton pattern to prevent accidental misuse.
  // To enable one MCU to use multiple AD411x peripherals at once some more
  // work is required to map hadrware interrupts to the correct instance.

#pragma region singleton pattern
// Region with singleton pattern implementation -------------------------------
  private:
    // Singleton pattern. First make the c-tors private.
    AD411x();

  public:
      // Aquire reference to the one and only instance, which is statically initialized.
      static AD411x &getInstance()
      {
        static AD411x instance; // Guaranteed to be destroyed. Instantiated on first use.
        return instance;
      }

      // Ensure no implementations for copy and assignment.
      // Make sure they are inaccessible (especially from outside), 
      // to prevent accidental copies of your singleton.
      // Dirrerent solution for C++ 11 onwards compares to legacy C++ 03.

      // C++ 11 and onwards
      // We can use this technique for deleting the methods we don't want.
      AD411x(AD411x const&)          = delete;
      void operator=(AD411x const&)  = delete;
// ----------------------------------------------------------------------------
#pragma endregion

	public:
    // Static configuration:
    static const bool DEBUG = false;  // Extra verbosity for debugging.   

    /*!
    @brief  Constructor/Initialization for an AD411x connected via native hardware SPI bus.
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) to enable communication with peripheral.
    @param  spi
            Pointer to an existing SPIClass instance (e.g. &SPI, the microcontroller's primary SPI bus).
    @param  bitrate
            SPI clock rate for transfers to this peripheral. Default value is 4000000UL (4 MHz).
    @note   Call the object's begin() function before use.
    */
    void setup(int8_t cs_pin, SPIClass *spi = &SPI, uint32_t bitrate = 4000000UL);
    //void setup(int8_t cs_pin);

    //dep. default parameters. AD411x(int8_t cs_pin, SPIClass *spi = &SPI, uint32_t bitrate = 4000000UL);
    //AD411x(int8_t cs_pin, SPIClass *spi, uint32_t bitrate);
    //AD411x(int8_t cs_pin, int8_t sclk_pin, int8_t miso_pin, int8_t mosi_pin);  //TODO: Not tested!
    //private AD411x(int8_t cs_pin); 


		bool begin();

    // Generic read function for any register (1 to 4 bytes).
    bool read_register(byte reg, byte *data, byte data_len) const;
    // Generic read function for 16-bit registers.
    uint16_t read_register_16bit(byte reg) const;

    // Write a 16-bit word to a register.
    bool write_register(byte reg, uint16_t data) const;
    // Write a 8-bit word to a register.
    bool write_register(byte reg, byte data) const;


    bool check_id();
    void read_data();
    void read_status();
    void read_many_things();

    bool get_data_stat() const;
    void set_data_stat(bool value);

    // V_REF must be configurable.
    double get_v_ref() const;
    void set_v_ref(double value);

    void reset();
    void read_volt(byte *out_channel, double *out_volt);

    // Channel configuration.
    void configure_channel(byte channel, AD4116::InputType input, byte setup, bool enable = true);
    bool is_channel_enabled(byte channel);
    void enable_channel(byte channel);
    void disable_channel(byte channel);

    // Configure setups 0 to 7
    // Helper functions to create a setup value from boolean parameters.
    uint16_t make_setup(bool bipolar, bool refbuf_p, bool refbuf_n,
                    bool input_buffer, bool internal_vref, bool low_level_ref);

    // Configure on of the 8 setups.
    // To set the proper bits in the register, either use the make_setup function
    // or combine the constants defined in AD4116::Setup.
    void configure_setup(byte setup_number, uint16_t setup_bits);

    // Read one of the 8 setups.
    uint16_t read_setup(byte setup_number);


    bool is_data_ready() const;

#pragma region Counters
// Counters just to help observe and debug interrupt behaviour. ---------------
  public:
    uint32_t get_interrupt_count();
    uint32_t get_skip_count();
    uint32_t get_drdy_count();
    uint32_t get_read_count();

    uint32_t get_interrupt_lap();
    uint32_t get_skip_lap();
    uint32_t get_drdy_lap();
    uint32_t get_read_lap();
// ----------------------------------------------------------------------------
#pragma endregion

	private:
    static const BusIOBitOrder bit_order = SPI_BITORDER_MSBFIRST;
    // The AD411x communicates with clock polarity = 1 and clock phase = 1 aka
    // SPI mode 3 data shifted out on falling clk and data sampled on rising clk.
    static const uint8_t data_mode = SPI_MODE3;

    double v_ref;
    double bipolar_factor;
    double unipolar_factor;

    Adafruit_SPIDevice *spi_dev = NULL;
    int8_t cs_pin;
    bool data_stat;

    // Write data of arbitrary length to a register.
    bool write_register(byte reg, byte *data, byte data_len) const;


#pragma region Interrupt handler
// Region with static code to handle hardware interrupts. ---------------------
  public:
    void enable_interrupt(int8_t interrupt_pin);
    void disable_interrupt();

  private:
    // Called by interrupt handler when data is available on this instance.
    void on_data_ready();
    void clear_data_ready();
    
    // Static members. Mainly to handle hardware interrupts.
    static int8_t interrupt_pin;
    static void on_interrupt();   

// ----------------------------------------------------------------------------
#pragma endregion

};

// Singleton instance declaration.
extern AD411x &ad4116;

#endif /* __Adopticum_AD411x_h */
