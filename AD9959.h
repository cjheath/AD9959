/*
 * Library for the AD9959 DDS chip
 *
 * Connect:
 * Your chosen Chip Enable pin	to AD9959 CS (downshift to 3v3)
 * Your chosen Reset pin	to AD9959 RESET (downshift to 3v3)
 * Your chosen "Update" pin	to AD9959 I/O_UPDATE (downshift to 3v3)
 * Your chosen Sweep pin	to Profile 0, 1, 2, 3 (downshift to 3v3)
 * SPI CLK (default: pin 13)	to AD9959 SCLK (downshift to 3v3)
 * SPI MOSI (default: pin 11)	to AD9959 SDIO_0 (downshift to 3v3)
 * Nothing (floating)		to AD9959 SDIO_1
 * SPI MISO (default: pin 12)	to AD9959 SDIO_2 (upshift from 3v3)
 * Ground			to AD9959 SDIO_3
 * Ground			to AD9959 Ground
 * 3v3				to AD9959 Digital Vcc
 * 3v3				to AD9959 Analog Vcc (+bead/filter)
 *
 * To calibrate, use the default calibration value (10MHz)
 * and measure the generated frequency with a frequency counter.
 * Then provide that value as the calibration constant instead.
 */

#ifndef _AD9959_h_
#define _AD9959_h_

#include "Arduino.h"
#include <SPI.h>

#if ARDUINO < 10600
#error "Arduino 1.6.0 or later (SPI library) is required"
#endif

template <
	uint8_t		ResetPin,		// Reset pin (active = high)
	uint8_t		ChipEnablePin,		// Chip Enable (active low)
	uint8_t		UpdatePin,		// I/O_UPDATE: Apply config changes
	unsigned long	calibration = 10000000,	// Use your actual frequency when set to 10MHz
	long		SPIRate = 2000000,	// 2MBit/s (16MHz Arduino can do 8Mhz)
	uint8_t		SPIClkPin = 13,		// Note: do not change the SPI pins, they're currently ignored.
	uint8_t		SPIMISOPin = 12,
	uint8_t		SPIMOSIPin = 11
>
class AD9959
{
public:
  typedef enum {
    ChannelNone	= 0x00,
    Channel0	= 0x10,
    Channel1	= 0x20,
    Channel2	= 0x40,
    Channel3	= 0x80,
    ChannelAll	= 0xF0,
  } ChannelNum;

  typedef enum {	// There are 334 bytes in all the registers! See why below...
    CSR               = 0x00,	// 1 byte, Channel Select Register
    FR1               = 0x01,	// 3 bytes, Function Register 1
    FR2               = 0x02,	// 2 bytes, Function Register 2
    				// The following registers are duplicated for each channel.
				// A write goes to any and all registers based on channel select (CSR)
    CFR               = 0x03,	// 3 bytes, Channel Function Register (one for each channel!)
    CFTW              = 0x04,	// 4 bytes, Channel Frequency Tuning Word
    CPOW              = 0x05,	// 2 bytes, Channel Phase Offset Word (aligned to LSB, top 2 bits unused)
    ACR               = 0x06,	// 3 bytes, Amplitude Control Register (rate byte, control byte, scale byte)
    LSRR              = 0x07,	// 2 bytes, Linear Sweep Rate Register (falling, rising)
    RDW               = 0x08,	// 4 bytes, Rising Delta Word
    FDW               = 0x09,	// 4 bytes, Falling Delta Word
    				// The following registers (per channel) are used to provide 16 modulation values
				// This library doesn't provide modulation. Only CW1 is used, for sweep destination.
    CW1               = 0x0A,	// 4 bytes, Channel Word 1-15 (phase & amplitude MSB aligned)
    CW2               = 0x0B,
    CW3               = 0x0C,
    CW4               = 0x0D,
    CW5               = 0x0E,
    CW6               = 0x0F,
    CW7               = 0x10,
    CW8               = 0x11,
    CW9               = 0x12,
    CW10              = 0x13,
    CW11              = 0x14,
    CW12              = 0x15,
    CW13              = 0x16,
    CW14              = 0x17,
    CW15              = 0x18,
    READ              = 0x80	// Pseudo-register to indicate read
  } Register;

  typedef struct {
    // Bit order selection (default MSB):
    static constexpr  uint8_t MSB_First = 0x00;
    static constexpr  uint8_t LSB_First = 0x01;
    // Serial I/O Modes (default IO2Wire):
    static constexpr  uint8_t IO2Wire = 0x00;
    static constexpr  uint8_t IO3Wire = 0x02;
    static constexpr  uint8_t IO2Bit = 0x04;
    static constexpr  uint8_t IO4Bit = 0x06;
  } CSR_Bits;

  typedef struct {    // Function Register 1 is 3 bytes wide.
    // Most significant byte:
    // Higher charge pump values decrease lock time and increase phase noise
    static constexpr  uint32_t ChargePump0	= 0x00;
    static constexpr  uint32_t ChargePump1	= 0x01;
    static constexpr  uint32_t ChargePump2	= 0x02;
    static constexpr  uint32_t ChargePump3	= 0x03;

    static constexpr  uint32_t PllDivider	= 0x04; // multiply 4..20 by this (or shift 19)
    static constexpr  uint32_t VCOGain		= 0x80; // Set low for VCO<160MHz, high for >255MHz

    // Middle byte:
    static constexpr  uint32_t ModLevels2	= 0x00; // How many levels of modulation?
    static constexpr  uint32_t ModLevels4	= 0x01;
    static constexpr  uint32_t ModLevels8	= 0x02;
    static constexpr  uint32_t ModLevels16	= 0x03;

    static constexpr  uint32_t RampUpDownOff	= 0x00;
    static constexpr  uint32_t RampUpDownP2P3	= 0x04; // Profile=0 means ramp-up, 1 means ramp-down
    static constexpr  uint32_t RampUpDownP3	= 0x08; // Profile=0 means ramp-up, 1 means ramp-down
    static constexpr  uint32_t RampUpDownSDIO123= 0x0C; // Only in 1-bit I/O mode

    static constexpr  uint32_t Profile0		= 0x00;
    static constexpr  uint32_t Profile7		= 0x07;

    // Least significant byte:
    static constexpr  uint32_t SyncAuto		= 0x00;	// Master SYNC_OUT->Slave SYNC_IN, with FR2
    static constexpr  uint32_t SyncSoft		= 0x01;	// Each time this is set, system clock slips one cycle
    static constexpr  uint32_t SyncHard		= 0x02;	// Synchronise devices by slipping on SYNC_IN signal

    // Software can power-down individual channels (using CFR[7:6])
    static constexpr  uint32_t DACRefPwrDown	= 0x10;	// Power-down DAC reference
    static constexpr  uint32_t SyncClkDisable	= 0x20;	// Don't output SYNC_CLK
    static constexpr  uint32_t ExtFullPwrDown	= 0x40;	// External power-down means full power-down (DAC&PLL)
    static constexpr  uint32_t RefClkInPwrDown	= 0x80;	// Disable reference clock input
  } FR1_Bits;

  typedef struct {
    static constexpr  uint32_t AllChanAutoClearSweep	= 0x8000;// Clear sweep accumulator(s) on I/O_UPDATE
    static constexpr  uint32_t AllChanClearSweep	= 0x4000;// Clear sweep accumulator(s) immediately
    static constexpr  uint32_t AllChanAutoClearPhase	= 0x2000;// Clear phase accumulator(s) on I/O_UPDATE
    static constexpr  uint32_t AllChanClearPhase	= 0x2000;// Clear phase accumulator(s) immediately
    static constexpr  uint32_t AutoSyncEnable	= 0x0080;
    static constexpr  uint32_t MasterSyncEnable	= 0x0040;
    static constexpr  uint32_t MasterSyncStatus	= 0x0020;
    static constexpr  uint32_t MasterSyncMask	= 0x0010;
    static constexpr  uint32_t SystemClockOffset = 0x0003;	// Mask for 2-bit clock offset controls
  } FR2_Bits;

  AD9959()
  {
    // Ensure that the SPI device is initialised
    // "setting SCK, MOSI, and SS to outputs, pulling SCK and MOSI low, and SS high"
    SPI.begin();

    pinMode(ResetPin, OUTPUT);		// Ensure we can reset the AD9959
    pinMode(ChipEnablePin, OUTPUT);	// This control signal applies the loaded values
    digitalWrite(ChipEnablePin, 1);
    pinMode(UpdatePin, OUTPUT);		// This control signal applies the loaded values
    digitalWrite(UpdatePin, 0);

    reset();
  }

  void reset()
  {
    pulse(ResetPin);			// (minimum 5 cycles of the 30MHz clock)
    pulse(SPIClkPin);			// Enable serial loading mode:
    pulse(UpdatePin);
    // Disable all channels, set 3-wire MSB mode:
    write(CSR, ChannelNone|CSR_Bits::MSB_First|CSR_Bits::IO3Wire);
    pulse(UpdatePin);
  }

  void setVCO(int mult = 20)	// Must be 0 or in range 4..20
  {
    spi_begin();
    SPI.transfer(FR1);
    // VCO Gain is needed for a 255-500MHz master clock:
    SPI.transfer(FR1_Bits::VCOGain | (mult*FR1_Bits::PllDivider) | FR1_Bits::ChargePump3);
    // Profile0 means each channel is modulated by a different profile pin:
    SPI.transfer(FR1_Bits::ModLevels2 | FR1_Bits::RampUpDownOff | FR1_Bits::Profile0);
    SPI.transfer(FR1_Bits::SyncClkDisable); // Don't output SYNC_CLK
    spi_end();
  }

protected:
  void pulse(uint8_t pin)
  {
    raise(pin);
    lower(pin);
  }

  void lower(uint8_t pin)
  {
    digitalWrite(pin, LOW);
  }

  void raise(uint8_t pin)
  {
    digitalWrite(pin, HIGH);
  }

  void chip_enable()
  {
    lower(ChipEnablePin);
  }

  void chip_disable()
  {
    raise(ChipEnablePin);
  }

  void spi_begin()
  {
    SPI.beginTransaction(SPISettings(SPIRate, LSBFIRST, SPI_MODE0));
    chip_enable();
  }

  void spi_end()
  {
    chip_disable();
    SPI.endTransaction();
  }

  void write(Register reg, byte value)
  {
    spi_begin();
    SPI.transfer(reg);
    SPI.transfer(value);
    spi_end();
  }

  uint8_t read(Register reg)
  {
    byte  value = 0;
    spi_begin();
    SPI.transfer(READ | reg);
    value = SPI.transfer(0);
    spi_end();
    return value;
  }

  void write(Register reg, byte* buffer, int len)
  {
    spi_begin();
    SPI.transfer(reg);
    SPI.transfer(buffer,len);
    spi_end();
  }

  void read(Register reg, byte* buffer, int len)
  {
    spi_begin();
    SPI.transfer(READ | reg);
    SPI.transfer(buffer, len);
    spi_end();
  }
};

#endif  /* _AD9959_h_ */
