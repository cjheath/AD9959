/*
 * Library for the AD9959 DDS chip
 *
 * Connect:
 * Your chosen Chip Enable pin  to AD9959 CS (downshift to 3v3)
 * Your chosen Reset pin        to AD9959 RESET (downshift to 3v3)
 * Your chosen "Update" pin     to AD9959 I/O_UPDATE (downshift to 3v3)
 * Your chosen Sweep pin        to Profile 0, 1, 2, 3 (downshift to 3v3)
 * SPI CLK (default: pin 13)    to AD9959 SCLK (downshift to 3v3)
 * SPI MOSI (default: pin 11)   to AD9959 SDIO_0 (downshift to 3v3)
 * Nothing (floating)           to AD9959 SDIO_1
 * SPI MISO (default: pin 12)   to AD9959 SDIO_2 (upshift from 3v3)
 * Ground                       to AD9959 SDIO_3
 * Ground                       to AD9959 Ground
 * 3v3                          to AD9959 Digital Vcc
 * 3v3                          to AD9959 Analog Vcc (+bead/filter)
 *
 * To calibrate, use a zero calibration value, set some frequency and
 * measure it as accurately as you can with a frequency counter.
 * Convert that value to parts-per-billion error (positive if the
 * frequency is high, negative if low) and provide that value to
 * setClock() as the calibration value instead. This will calculate
 * the actual core frequency and base deltas off that.
 */

#ifndef _AD9959_h_
#define _AD9959_h_

#include "Arduino.h"
#include <SPI.h>

#if ARDUINO < 10600
#error "Arduino 1.6.0 or later (SPI library) is required"
#endif

#if     defined(DDS_MAX_PRECISION)
#if     !defined(MAX_U64)
#define MAX_U64 ((uint64_t)~0LL)
#endif
#endif

template <
        uint8_t         ResetPin,               // Reset pin (active = high)
        uint8_t         ChipEnablePin,          // Chip Enable (active low)
        uint8_t         UpdatePin,              // I/O_UPDATE: Apply config changes
        unsigned long   reference_freq = 25000000, // Use your crystal or reference frequency
        long            SPIRate = 2000000,      // 2MBit/s (16MHz Arduino can do 8Mhz)
        uint8_t         SPIClkPin = 13,         // Note: do not change the SPI pins, they're currently ignored.
        uint8_t         SPIMISOPin = 12,
        uint8_t         SPIMOSIPin = 11
>
class AD9959
{
  uint32_t              core_clock;             // reference_freq*pll_mult after calibration
#if     defined(DDS_MAX_PRECISION)
  uint64_t              reciprocal;             // (2^64-1)/core_clock
#else
  uint32_t              reciprocal;             // 2^(64-shift)/core_clock
  uint8_t               shift;                  // (2<<shift) < core_clock, but just (28 or less)
#endif
  uint8_t               last_channels;

public:
  typedef enum {
    ChannelNone = 0x00,
    Channel0    = 0x10,
    Channel1    = 0x20,
    Channel2    = 0x40,
    Channel3    = 0x80,
    ChannelAll  = 0xF0,
  } ChannelNum;

  // See register_length[] in write() before re-ordering these.
  typedef enum {        // There are 334 bytes in all the registers! See why below...
    CSR               = 0x00,   // 1 byte, Channel Select Register
    FR1               = 0x01,   // 3 bytes, Function Register 1
    FR2               = 0x02,   // 2 bytes, Function Register 2
                                // The following registers are duplicated for each channel.
                                // A write goes to any and all registers enabled in channel select (CSR)
                                // To read successfully you must first select one channel
    CFR               = 0x03,   // 3 bytes, Channel Function Register (one for each channel!)
    CFTW              = 0x04,   // 4 bytes, Channel Frequency Tuning Word
    CPOW              = 0x05,   // 2 bytes, Channel Phase Offset Word (aligned to LSB, top 2 bits unused)
    ACR               = 0x06,   // 3 bytes, Amplitude Control Register (rate byte, control byte, scale byte)
    LSRR              = 0x07,   // 2 bytes, Linear Sweep Rate Register (falling, rising)
    RDW               = 0x08,   // 4 bytes, Rising Delta Word
    FDW               = 0x09,   // 4 bytes, Falling Delta Word
                                // The following registers (per channel) are used to provide 16 modulation values
                                // This library doesn't provide modulation. Only CW1 is used, for sweep destination.
    CW1               = 0x0A,   // 4 bytes, Channel Word 1-15 (phase & amplitude MSB aligned)
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
    CW15              = 0x18
  } Register;

  typedef enum {
    // Bit order selection (default MSB):
    MSB_First = 0x00,
    LSB_First = 0x01,
    // Serial I/O Modes (default IO2Wire):
    IO2Wire = 0x00,
    IO3Wire = 0x02,
    IO2Bit = 0x04,
    IO4Bit = 0x06,
  } CSR_Bits;

  typedef enum {    // Function Register 1 is 3 bytes wide.
    // Most significant byte:
    // Higher charge pump values decrease lock time and increase phase noise
    ChargePump0      = 0x00,
    ChargePump1      = 0x01,
    ChargePump2      = 0x02,
    ChargePump3      = 0x03,

    PllDivider       = 0x04,    // multiply 4..20 by this (or shift 19)
    VCOGain          = 0x80,    // Set low for VCO<160MHz, high for >255MHz

    // Middle byte:
    ModLevels2       = 0x00,    // How many levels of modulation?
    ModLevels4       = 0x01,
    ModLevels8       = 0x02,
    ModLevels16      = 0x03,

    RampUpDownOff    = 0x00,    // Which pins contol amplitude ramping?
    RampUpDownP2P3   = 0x04,    // Profile=0 means ramp-up, 1 means ramp-down
    RampUpDownP3     = 0x08,    // Profile=0 means ramp-up, 1 means ramp-down
    RampUpDownSDIO123= 0x0C,    // Only in 1-bit I/O mode

    Profile0         = 0x00,
    Profile7         = 0x07,

    // Least significant byte:
    SyncAuto         = 0x00,    // Master SYNC_OUT->Slave SYNC_IN, with FR2
    SyncSoft         = 0x01,    // Each time this is set, system clock slips one cycle
    SyncHard         = 0x02,    // Synchronise devices by slipping on SYNC_IN signal

    // Software can power-down individual channels (using CFR[7:6])
    DACRefPwrDown    = 0x10,    // Power-down DAC reference
    SyncClkDisable   = 0x20,    // Don't output SYNC_CLK
    ExtFullPwrDown   = 0x40,    // External power-down means full power-down (DAC&PLL)
    RefClkInPwrDown  = 0x80,    // Disable reference clock input
  } FR1_Bits;

  typedef enum {
    AllChanAutoClearSweep    = 0x8000,  // Clear sweep accumulator(s) on I/O_UPDATE
    AllChanClearSweep        = 0x4000,  // Clear sweep accumulator(s) immediately
    AllChanAutoClearPhase    = 0x2000,  // Clear phase accumulator(s) on I/O_UPDATE
    AllChanClearPhase        = 0x2000,  // Clear phase accumulator(s) immediately
    AutoSyncEnable   = 0x0080,
    MasterSyncEnable = 0x0040,
    MasterSyncStatus = 0x0020,
    MasterSyncMask   = 0x0010,
    SystemClockOffset = 0x0003,         // Mask for 2-bit clock offset controls
  } FR2_Bits;

  // Channel Function Register
  typedef enum {
    ModulationMode   = 0xC00000,        // Mask for modulation mode
    AmplitudeModulation = 0x400000,     // Mask for modulation mode
    FrequencyModulation = 0x800000,     // Mask for modulation mode
    PhaseModulation  = 0xC00000,        // Mask for modulation mode
    SweepNoDwell     = 0x008000,        // No dwell mode
    SweepEnable      = 0x004000,        // Enable the sweep
    SweepStepTimerExt = 0x002000,       // Reset the sweep step timer on I/O_UPDATE
    DACFullScale     = 0x000300,        // 1/8, 1/4, 1/2 or full DAC current
    DigitalPowerDown = 0x000080,        // Power down the DDS core
    DACPowerDown     = 0x000040,        // Power down the DAC
    MatchPipeDelay   = 0x000020,        // Compensate for pipeline delays
    AutoclearSweep   = 0x000010,        // Clear the sweep accumulator on I/O_UPDATE
    ClearSweep       = 0x000008,        // Clear the sweep accumulator immediately
    AutoclearPhase   = 0x000004,        // Clear the phase accumulator on I/O_UPDATE
    ClearPhase       = 0x000002,        // Clear the phase accumulator immediately
    OutputSineWave   = 0x000001,        // default is cosine
  } CFR_Bits;

  // Amplitude Control Register
  typedef enum {
    RampRate            = 0xFF0000,     // Time between ramp steps
    StepSize            = 0x00C000,     // Amplitude step size (00=1,01=2,10=4,11=8)
    MultiplierEnable    = 0x001000,     // 0 means bypass the amplitude multiplier
    RampEnable          = 0x000800,     // 0 means aplitude control is manual
    LoadARRAtIOUpdate   = 0x000400,     // Reload Amplitude Rate Register at I/O Update
    ScaleFactor         = 0x0003FF,     // 10 bits for the amplitude target
  } ACR_Bits;

  AD9959()
  : core_clock(0)
  , last_channels(0xF0)
  {
    // Ensure that the SPI device is initialised
    // "setting SCK, MOSI, and SS to outputs, pulling SCK and MOSI low, and SS high"
    SPI.begin();

    digitalWrite(ResetPin, 0);
    pinMode(ResetPin, OUTPUT);          // Ensure we can reset the AD9959
    digitalWrite(ChipEnablePin, 1);
    pinMode(ChipEnablePin, OUTPUT);     // This control signal applies the loaded values
    digitalWrite(UpdatePin, 0);
    pinMode(UpdatePin, OUTPUT);         // This control signal applies the loaded values

    reset();
  }

  /*
   * Reset, applying CFR bits requested.
   * You might want to add:
   * CFR_Bits::AutoclearSweep - clear the sweep accumulator on I/O update or profile change
   * CFR_Bits::AutoclearPhase - clear the phase accumulator on I/O update or profile change
   */
  void reset(
    CFR_Bits cfr =
      CFR_Bits::DACFullScale |
      CFR_Bits::MatchPipeDelay |
      CFR_Bits::OutputSineWave
  )
  {
    pulse(ResetPin);                    // (minimum 5 cycles of the 30MHz clock)
    pulse(SPIClkPin);                   // Enable serial loading mode:
    pulse(UpdatePin);

    // Apply the requested CFR bits
    last_channels = ChannelNone;        // Ensure channels get set, not optimised out
    setChannels(ChannelAll);
    write(CFR, cfr);

    setChannels(ChannelNone);           // Disable all channels, set 3-wire MSB mode:
    pulse(UpdatePin);                   // Apply the changes
    setClock();                         // Set the PLL going
    // It will take up to a millisecond before the PLL locks and stabilises.
  }

  void setClock(int mult = 20, int32_t calibration = 0) // Mult must be 0 or in range 4..20
  {
    if (mult < 4 || mult > 20)
      mult = 1;                         // Multiplier is disabled.
    core_clock = reference_freq * (1000000000ULL+calibration) / 1000000000ULL * mult;
#if defined(DDS_MAX_PRECISION)
    reciprocal = MAX_U64 / core_clock;
#else
    // The AVR gcc implementation has a 32x32->64 widening multiply.
    // This is quite accurate enough, and considerably faster than full 64x64.
    uint64_t    scaled = core_clock;
    for (shift = 32; shift > 0 && (scaled&0x100000000ULL) == 0; shift--)
      scaled <<= 1;                   // Ensure that reciprocal fits in 32 bits
    reciprocal = (0x1ULL<<(32+shift)) / core_clock;
#endif
    // Serial.print("core_clock="); Serial.println(core_clock);
    // Serial.print("reciprocal="); Serial.println(reciprocal);
    spiBegin();
    SPI.transfer(FR1);
    // High VCO Gain is needed for a 255-500MHz master clock, and not up to 160Mhz
    // In-between is unspecified.
    SPI.transfer(
      (core_clock > 200 ? FR1_Bits::VCOGain : 0)
      | (mult*FR1_Bits::PllDivider)
      | FR1_Bits::ChargePump3         // Lock fast
    );
    // Profile0 means each channel is modulated by a different profile pin:
    SPI.transfer(FR1_Bits::ModLevels2 | FR1_Bits::RampUpDownOff | FR1_Bits::Profile0);
    SPI.transfer(FR1_Bits::SyncClkDisable); // Don't output SYNC_CLK
    spiEnd();
  }

  // Calculating deltas is expensive. You might use this infrequently and then use setDelta
  uint32_t frequencyDelta(uint32_t freq) const
  {
#if defined(DDS_MAX_PRECISION)
    return (freq * reciprocal + 0x80000000UL) >> 32;
#else
    // The reciprocal/16 is a rounding factor determined experimentally
    return ((uint64_t)freq * reciprocal + reciprocal/16) >> shift;
#endif
  }

  void setFrequency(ChannelNum chan, uint32_t freq)
  {
    setDelta(chan, frequencyDelta(freq));
  }

  void setDelta(ChannelNum chan, uint32_t delta)
  {
    setChannels(chan);
    write(CFTW, delta);
  }

  void setAmplitude(ChannelNum chan, uint16_t amplitude)        // Maximum amplitude value is 1024
  {
    if (amplitude > 1024)
      amplitude = 1024;                 // Clamp to the maximum
    setChannels(chan);
    spiBegin();
    SPI.transfer(ACR);                  // Amplitude control register
    SPI.transfer(0);                    // Time between ramp steps
    if (amplitude < 1024)               // Enable amplitude control with no ramping
      SPI.transfer((ACR_Bits::MultiplierEnable | amplitude)>>8);
    else
      SPI.transfer(0);                  // Disable the amplitude multiplier
    SPI.transfer(amplitude&0xFF);       // Bottom 8 bits of amplitude
    spiEnd();
  }

  void setPhase(ChannelNum chan, uint16_t phase)                // Maximum phase value is 16383
  {
    setChannels(chan);
    write(CPOW, phase & 0x3FFF);        // Phase wraps around anyway
  }

  void update()
  {
    pulse(UpdatePin);
  }

  void sweepFrequency(ChannelNum chan, uint32_t freq, bool follow = true)       // Target frequency
  {
    sweepDelta(chan, frequencyDelta(freq), follow);
  }

  void sweepDelta(ChannelNum chan, uint32_t delta, bool follow = true)
  {
    setChannels(chan);
    // Set up for frequency sweep
    write(
      CFR,
      CFR_Bits::FrequencyModulation |
      CFR_Bits::SweepEnable |
      CFR_Bits::DACFullScale |
      CFR_Bits::MatchPipeDelay |
      (follow ? 0 : CFR_Bits::SweepNoDwell)
    );
    // Write the frequency delta into the sweep destination register
    write(CW1, delta);
  }

  void sweepAmplitude(ChannelNum chan, uint16_t amplitude, bool follow = true)  // Target amplitude (half)
  {
    setChannels(chan);

    // Set up for amplitude sweep
    write(
      CFR,
      CFR_Bits::AmplitudeModulation |
      CFR_Bits::SweepEnable |
      CFR_Bits::DACFullScale |
      CFR_Bits::MatchPipeDelay |
      (follow ? 0 : CFR_Bits::SweepNoDwell)
    );

    // Write the amplitude into the sweep destination register, MSB aligned
    write(CW1, ((uint32_t)amplitude) * (0x1<<(32-10)));
  }

  void sweepPhase(ChannelNum chan, uint16_t phase, bool follow = true)          // Target phase (180 degrees)
  {
    setChannels(chan);

    // Set up for phase sweep
    write(
      CFR,
      CFR_Bits::PhaseModulation |
      CFR_Bits::SweepEnable |
      CFR_Bits::DACFullScale |
      CFR_Bits::MatchPipeDela |
      (follow ? 0 : CFR_Bits::SweepNoDwell)
    );

    // Write the phase into the sweep destination register, MSB aligned
    write(CW1, ((uint32_t)phase) * (0x1<<(32-14)));
  }

  void sweepRates(ChannelNum chan, uint32_t increment, uint8_t up_rate, uint32_t decrement = 0, uint8_t down_rate = 0)
  {
    setChannels(chan);
    write(RDW, increment);                      // Rising Sweep Delta Word
    write(FDW, increment);                      // Falling Sweep Delta Word
    write(LSRR, (down_rate<<8) | up_rate);      // Linear Sweep Ramp Rate
  }

  void setChannels(ChannelNum chan)
  {
    if (last_channels != chan)
      write(CSR, chan|CSR_Bits::MSB_First|CSR_Bits::IO3Wire);
    last_channels = chan;
  }

  // To read channel registers, you must first use setChannels to select exactly one channel!
  uint32_t read(Register reg)
  {
    return write(0x80|reg, 0);  // The zero data is discarded, just the return value is used
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

  void chipEnable()
  {
    lower(ChipEnablePin);
  }

  void chipDisable()
  {
    raise(ChipEnablePin);
  }

  void spiBegin()
  {
    SPI.beginTransaction(SPISettings(SPIRate, MSBFIRST, SPI_MODE3));
    chipEnable();
  }

  void spiEnd()
  {
    chipDisable();
    SPI.endTransaction();
  }

  // Read or write the specified register (0x80 bit means read)
  uint32_t write(uint8_t reg, uint32_t value)
  {
    // The indices of this array match the values of the Register enum:
    static constexpr uint8_t register_length[8] = { 1, 3, 2, 3, 4, 2, 3, 2 };  // And 4 beyond that

    uint32_t    rval = 0;
    int         len = (reg&0x7F) < sizeof(register_length)/sizeof(uint8_t) ? register_length[reg&0x07] : 4;
    spiBegin();
    SPI.transfer(reg);
    while (len-- > 0)
      rval = (rval<<8) | SPI.transfer((value>>len*8) & 0xFF);
    spiEnd();
    return rval;
  }

};

#endif  /* _AD9959_h_ */
