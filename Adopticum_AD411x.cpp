/*
Adopticum_AD411x
AD411x Analog to digital converter Arduino library

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#include <Arduino.h>
#include <Adafruit_SPIDevice.h>
#include <SPI.h>
#include "Adopticum_AD411x.h"
#include "Adopticum_AD411x_Definitions.h"
#include "AD4116_Input.h"
#include "AD4116_Setup.h"
#include "Helpers.h"

using namespace Helpers;


#pragma region static counters
// Counters just to help observe and debug interrupt behaviour. ---------------

// Variables accessed from interrupt handler must be declared volatile.
volatile uint32_t interrupt_count[2];
volatile uint32_t skip_count[2];
volatile uint32_t drdy_count[2];
uint32_t read_count[2];
uint32_t drdy_previous;

void reset_static_counters()
{
  interrupt_count[0] = 0;
  interrupt_count[1] = 0;
  skip_count[0] = 0;
  skip_count[1] = 0;
  drdy_count[0] = 0;
  drdy_count[1] = 0;
  read_count[0] = 0;
  read_count[1] = 0;
  drdy_previous = 0;
}

// Get the count for this lap and start a new lap.
int32_t get_and_set_lap(volatile uint32_t *pair) 
{
  int32_t delta = pair[0] - pair[1];
  pair[1] = pair[0];
  return delta;
}

uint32_t AD411x::get_interrupt_count() { return interrupt_count[0]; }
uint32_t AD411x::get_skip_count() { return skip_count[0]; }
uint32_t AD411x::get_drdy_count() { return drdy_count[0]; }
uint32_t AD411x::get_read_count() { return read_count[0]; }

uint32_t AD411x::get_interrupt_lap() { return get_and_set_lap(interrupt_count); }
uint32_t AD411x::get_skip_lap() { return get_and_set_lap(skip_count); }
uint32_t AD411x::get_drdy_lap() { return get_and_set_lap(drdy_count); }
uint32_t AD411x::get_read_lap() { return get_and_set_lap(read_count); }

// ----------------------------------------------------------------------------
#pragma endregion


#pragma region initialization
// Region with constructors and initialization --------------------------------

// Singleton pattern implementation. Initialize the static instance.
AD411x &ad4116 = AD411x::getInstance();

// Define and initialize static variables.
int8_t AD411x::interrupt_pin = -1;
volatile bool interrupt_skip = false;

// private
// Constructor is never called from outside.
// Initialize member variables with default values.
AD411x::AD411x()
{
  set_v_ref(2.5);
}

/*!
    @brief  Constructor/Initialization for an AD411x connected via native hardware SPI bus.
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) for sharing the bus with other devices. Active low.
    @param  spi
            Pointer to an existing SPIClass instance (e.g. &SPI, the microcontroller's primary SPI bus).
    @param  bitrate
            SPI clock rate for transfers to this display. Default value is 4000000UL (4 MHz).
    @note   Call the object's begin() function before use.
*/
void AD411x::setup(int8_t cs_pin, SPIClass *spi, uint32_t bitrate)
{
  reset_static_counters();
  this->cs_pin = cs_pin;
  if (spi_dev)
  {
    delete spi_dev;
  }
  spi_dev = new Adafruit_SPIDevice(cs_pin, bitrate, AD411x::bit_order, AD411x::data_mode, spi);
}

// Default is to use the primary SPI bus at 4 MHz.
//void AD411x::setup(int8_t cs_pin)
//{ setup(cs_pin, &SPI, 4000000UL); }


//TODO: Not needed so far. Not tested.
// Communicate over software bitbang SPI.
// AD411x::AD411x(int8_t cs_pin, int8_t sclk_pin, int8_t miso_pin, int8_t mosi_pin)
//   : AD411x(cs_pin)
// {
//   spi_dev = new Adafruit_SPIDevice(cs_pin, sclk_pin, miso_pin, mosi_pin, 
//     1000000, AD411x::bit_order, AD411x::data_mode);
// }


bool AD411x::begin()
{
  if (!spi_dev->begin()) {
    return false;
  } 

  //TODO: Verify that AD411x is connected by reading the ID register.
  return true;
}
// ----------------------------------------------------------------------------
#pragma endregion

bool AD411x::read_register(byte reg, byte *data, byte data_len) const
{
  // Write the registry address to the comms register. Then read the data back.
  // bit 7 !WEN Must be 0.
  // bit 6 R/W Set to 1 to read the register.
  // bits 5..0 Registry address.
  byte addr = 0x3F & reg;
  data[0] = 0x40 | addr;
  interrupt_skip = true;
  spi_dev->write_then_read(data, 1, data, data_len);
  interrupt_skip = false;

  // Pull and hold CS pin low to get interrupt on DOUT/DRDY when data is ready.
  digitalWrite(this->cs_pin, LOW);

  if (this->DEBUG) {
    Serial.print("Read register 0x");
    print_bytes(&addr, 1);
    Serial.print(": 0x");
    print_bytes(data, data_len);
    Serial.println(".");
  }
  return true;
}

uint16_t AD411x::read_register_16bit(byte reg) const
{
  // Write the registry address to the comms register. Then read the data back.
  // bit 7 !WEN Must be 0.
  // bit 6 R/W Set to 1 to read the register.
  // bits 5..0 Registry address.
  byte data[2];
  byte addr = 0x40 | (0x3F & reg);
  interrupt_skip = true;
  spi_dev->write_then_read(&addr, 1, data, 2);
  interrupt_skip = false;
  // Pull and hold CS pin low to get interrupt on DOUT/DRDY when data is ready.
  digitalWrite(this->cs_pin, LOW);
  // Data as one 16 bit word, MSB first.
  return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}


bool AD411x::write_register(byte reg, byte *data, byte data_len) const
{
  // Write the registry address to the comms register. Then write the data to the register.
  // bit 7 !WEN Must be 0.
  // bit 6 R/W Set to 0 to write to the register.
  // bits 5..0 Registry address.
  byte addr = 0x3F & reg;
  interrupt_skip = true;
  spi_dev->write(data, data_len, &addr, 1);
  interrupt_skip = false;

  // Pull and hold CS pin low to get interrupt on DOUT/DRDY when data is ready.
  digitalWrite(this->cs_pin, LOW);

  if (this->DEBUG) {
    Serial.print("Write register 0x");
    print_bytes(&addr, 1);
    Serial.print(": 0x");
    print_bytes(data, data_len);
    Serial.println(".");
  }
}

// Write a 16-bit word to a register.
bool AD411x::write_register(byte reg, uint16_t data) const
{
  // Order the data MSB first in a buffer.
  byte buf[2];
  buf[1] = (byte) (data & 0xff);
  buf[0] = (byte) ((data >> 8) & 0xff);
  return write_register(reg, buf, 2);
}

// Write a 8-bit word to a register.
bool AD411x::write_register(byte reg, byte data) const
{
  return write_register(reg, &data, 1);
}


bool AD411x::check_id()
{ // The ID register returns a 16-bit ID. For the AD4116, this value is 0x34Dx. (x=undefined)
  byte buf[4];
  read_register(0x07, buf, 2);
  bool is_ad4116 = (0x34 == buf[0]) && (0xd0 == (0xf0 & buf[1]));
  if (this->DEBUG) {
		if (is_ad4116) {
			Serial.println("ADC product id matches AD4116.");
		} else {
			Serial.print("ADC product id is unknown (0x");
			print_bytes(buf, 2);
			Serial.println(").");
		}
	}
  return is_ad4116;
}


// Convert the raw code value into volt.
// The range of output code is from code 000 ... 0 as the minimum value,
// 100 ... 0 at the mid-range value and 111 ... 1 as the maximum value.
// Conversion depends on unipolar or bipolar operation.
double code_to_volt_bipolar(int32_t code, double v_ref)
{
  // When the ADC is configured for bipolar operation, the output
  // code is offset binary with a negative full-scale voltage resulting
  // in a code of 000 … 000, a zero differential input voltage resulting in
  // a code of 100 … 000, and a positive full-scale input voltage
  // resulting in a code of 111 … 111.
  const double conv_factor = v_ref * 10.0 / (double)(0x00800000L);  // = 1.1920928955e-7
  auto volt = conv_factor * (code - 0x00800000L);
  return volt;
}

double code_to_volt_unipolar(int32_t code, double v_ref)
{
  // When the ADC is configured for unipolar operation, the
  // output code is natural (straight) binary with a zero differential
  // input voltage resulting in a code of 00 … 00, a midscale voltage
  // resulting in a code of 100 … 000, and a full-scale input voltage
  // resulting in a code of 111 … 111. 
  const double conv_factor = v_ref * 10.0 / (double)(0x00FFFFFFL);
  auto volt = conv_factor * code;
  return volt;
}


void AD411x::read_data()
{ // The data register (0x04) contains the ADC conversion result (24 bits).
  // This register contains the ADC conversion result. If DATA_STAT is set in the
  // interface mode register, the status register is appended to this register when
  // read, making this a 32-bit register.
  byte buf[4];
  byte len = this->data_stat ? 4 : 3;
  read_register(0x04, buf, len);  

  if (this->DEBUG) {
    Serial.print("ADC conversion result (0x");
    print_bytes(buf, len);
    int32_t code = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2]);
    auto volt = code_to_volt_bipolar(code, 2.5);
    Serial.print(", volt: ");
    Serial.println(volt);
	}
}


void AD411x::read_volt(byte *out_channel, double *out_volt)
{ // The data register (0x04) contains the ADC conversion result (24 bits).
  // This register contains the ADC conversion result. If DATA_STAT is set in the
  // interface mode register, the status register is appended to this register when
  // read, making this a 32-bit register.
  byte buf[4];
  byte len = this->data_stat ? 4 : 3;
  read_register(0x04, buf, len);  

  // The 4th byte tells us which channel the sample is from. Without status byte we will always assume 0.
  *out_channel = len == 3 ? 0 : buf[3];
  int32_t code = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2]);

  //TODO: Must observe bipolar vs. unipolar configuration.
  //Conversion to volt in bipolar configuration.
  *out_volt = this->bipolar_factor * (code - 0x00800000);
  //Conversion to volt in bipolar configuration.
  //TODO: *out_volt = this->unipolar_factor * code;

  clear_data_ready();
  read_count[0]++;
}


void AD411x::read_status()
{ 
  byte buf[4];
  read_register(0x00, buf, 1);  
  if (this->DEBUG) {
    Serial.print("Status register (0x");
    print_bytes(buf, 1);
    Serial.println(").");
	}
}

void AD411x::read_many_things()
{ 
  byte buf[4];
  read_register(0x00, buf, 1);  // Status
  Serial.print("Status register (0x");
  print_bytes(buf, 1);
  Serial.println(").");

  read_register(0x01, buf, 2);  // ADC mode  
  Serial.print("ADC mode register (0x");
  print_bytes(buf, 2);
  Serial.println(").");

  read_register(0x02, buf, 2);  // IF mode
  Serial.print("IF mode register (0x");
  print_bytes(buf, 2);
  Serial.println(").");

  read_register(0x03, buf, 3);  // Reg check  
  Serial.print("Reg check register (0x");
  print_bytes(buf, 3);
  Serial.println(").");

  read_register(0x04, buf, 3);  // Data
  Serial.print("Data register (0x");
  print_bytes(buf, 3);
  Serial.println(").");

  read_register(0x06, buf, 2);  // GPIOCON  
  Serial.print("GPIO config register (0x");
  print_bytes(buf, 2);
  Serial.println(").");

  read_register(0x07, buf, 2);  // ID
  Serial.print("ID register (0x");
  print_bytes(buf, 2);
  Serial.println(").");
}


bool AD411x::get_data_stat() const
{
  byte data[2];
  read_register(Registers::IFMODE, data, 2);  // IF mode
  bool value = (data[1] & 0x40) >> 6;
  return value; 
}


void AD411x::set_data_stat(bool value)
{
  byte data[2];
  read_register(0x02, data, 2);  // IF mode
  data[1] = set_bit(data[1], 6, value);
  write_register(0x02, data, 2);
  this->data_stat = value;
}

double AD411x::get_v_ref() const
{ return this->v_ref; }

void AD411x::set_v_ref(double value)
{ 
  this->v_ref = value; 
  bipolar_factor = value * 10.0 / (double)(0x00800000L);  // = 1.1920928955e-7
  unipolar_factor = value * 10.0 / (double)(0x00FFFFFFL);  //  
}


void AD411x::reset()
{
  // After a power-up cycle and when the power supplies are
  // stable, a device reset is required. In situations where interface
  // synchronization is lost, a device reset is also required. A write
  // operation of at least 64 serial clock cycles with DIN high returns
  // the ADC to the default state by resetting the entire device,
  // including the register contents. Alternatively, if CS is being used
  // with the digital interface, returning CS high sets the digital
  // interface to the default state and halts any serial interface operation. 
  //TODO: 64 cycles @ 4 Mhz = 16 us, hence bring CS high for 1 ms (60x longer) should be enough.

  digitalWrite(this->cs_pin, HIGH);  //spi_dev->setChipSelect(HIGH);
  delay(1);
  // Pull and keep CS low to use DOUT/RDY pin for interrupt when data is ready.
  digitalWrite(this->cs_pin, LOW);  //spi_dev->setChipSelect(LOW);
}


#pragma region Interrupt handler
// Region with static code to handle hardware interrupts. ---------------------

void AD411x::enable_interrupt(int8_t interrupt_pin)
{
  // Use one (interrupt supporting) pin for data ready interrupt.
  AD411x::interrupt_pin = interrupt_pin;
  interrupt_skip = false;
  reset_static_counters();
  if (AD411x::interrupt_pin >= 0)
  {
    pinMode(AD411x::interrupt_pin, INPUT_PULLUP);
    auto interruptNumber = digitalPinToInterrupt(AD411x::interrupt_pin);
    attachInterrupt(interruptNumber, AD411x::on_interrupt, FALLING);

    // Pull and hold CS pin low to get interrupt on DOUT/DRDY when data is ready.
    pinMode(this->cs_pin, OUTPUT);
    digitalWrite(this->cs_pin, LOW);
  }
}


void AD411x::disable_interrupt()
{
  if (AD411x::interrupt_pin >= 0) {
    detachInterrupt(digitalPinToInterrupt(AD411x::interrupt_pin));
    AD411x::interrupt_pin = -1;
  }
}


//member function called from the static interrupt handler.
void AD411x::on_data_ready()
{ }


 // Check the data ready condition.
bool AD411x::is_data_ready() const
{
  return drdy_count[0] != drdy_previous;
}


// Clear the data ready condition.
void AD411x::clear_data_ready()
{
  drdy_previous = drdy_count[0];
}


// static method that receives the hardware interrupt.
void AD411x::on_interrupt()
{
  interrupt_count[0]++;
  if (interrupt_skip) { 
    skip_count[0]++; 
    return; 
  }
  // Maps every interrupt to the singleton instance.
  drdy_count[0]++;
  //ad4116.on_data_ready();
  //TODO: Map each interrupt to its corresponding instance via the chip select status.
}


// Data Ready behaviour (from AD4116 data sheet)
// Serial Data Output/Data Ready Output. This pin serves a dual purpose. It functions as a serial data
// output pin to access the output shift register of the ADC. The output shift register can contain data
// from any of the on-chip data or control registers. The data-word/control word information is placed on
// the DOUT/RDY pin on the SCLK falling edge and is valid on the SCLK rising edge. When CS is high, the
// DOUT/RDY output is tristated. When CS is low, and a register is not being read, DOUT/RDY operates as
// a data ready pin, going low to indicate the completion of a conversion. If the data is not read after 
// the conversion, the pin goes high before the next update occurs. The DOUT/RDY falling edge can be used
// as an interrupt to a processor, indicating that valid data is available.

// ----------------------------------------------------------------------------
#pragma endregion


#pragma region Channel configuration
// Region with channel configuration ------------------------------------------

void AD411x::configure_channel(byte channel_number, AD4116::InputType input, byte setup, bool enable)
{
  if (channel_number > 15) { return; }
  // CH_EN, bit 15.
  uint16_t ch_en = enable ? 0x8000 : 0x0000;
  // SETUP, bits 14..12.
  uint16_t setup_sel = (setup & 0x7) << 12;
  // INPUT, bits 9..0.
  uint16_t ch_cfg = ch_en | setup_sel | (uint16_t)input;
  ad4116.write_register(Registers::CH0 + channel_number, ch_cfg);
}


bool AD411x::is_channel_enabled(byte channel_number)
{
  if (channel_number > 15) { return false; }
  auto ch_reg = read_register_16bit(Registers::CH0 + channel_number);
  return (ch_reg & 0x8000) != 0;  // Check if bit 15 is set.
}


void AD411x::enable_channel(byte channel_number)
{
  if (channel_number > 15) { return; }
  auto ch_reg = read_register_16bit(Registers::CH0 + channel_number);
  ch_reg |= 0x8000;  // Set bit 15.
  write_register(Registers::CH0 + channel_number, ch_reg);
}

void AD411x::disable_channel(byte channel_number)
{
  if (channel_number > 15) { return; }
  auto ch_reg = read_register_16bit(Registers::CH0 + channel_number);
  ch_reg &= 0x7FFF;  // Clear bit 15.
  write_register(Registers::CH0 + channel_number, ch_reg);
}

// ----------------------------------------------------------------------------
#pragma endregion

#pragma region Setup configuration
// Region with setup configuration --------------------------------------------

uint16_t AD411x::make_setup(bool bipolar, bool refbuf_p, bool refbuf_n, 
  bool input_buffer, bool internal_vref, bool low_level_ref)
{
  uint16_t setup = bipolar ? AD4116::Setup::BIPOLAR : AD4116::Setup::UNIPOLAR;
  if (refbuf_p) { setup |= AD4116::Setup::REFBUF_P; }
  if (refbuf_n) { setup |= AD4116::Setup::REFBUF_N; }
  if (input_buffer) { setup |= AD4116::Setup::INPUT_BUFFERS; }
  if (low_level_ref) { 
    setup |= AD4116::Setup::AVDD_AVSS_REF; 
  } else if (internal_vref) { 
    setup |= AD4116::Setup::INTERNAL_REF;
    //Note: Internal reference (2.5V). Must also be enabled in via ADCMODE.
  }
  return setup;
}

// Configure the setup register.
// To set the proper bits in the register, either use the make_setup function
// or combine the constants defined in AD4116::Setup.
void AD411x::configure_setup(byte setup_number, uint16_t setup_bits) 
{
  if (setup_number > 7) { return; }
  ad4116.write_register(Registers::SETUP0 + setup_number, setup_bits);
}


uint16_t AD411x::read_setup(byte setup_number)
{
  if (setup_number > 7) { return 0x0000; }
  auto setup_bits = read_register_16bit(Registers::SETUP0 + setup_number);
  return setup_bits;
}

//TODO: A method to parse fields out of setup register (setup_bits).

// ----------------------------------------------------------------------------
#pragma endregion

