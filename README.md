# AD9959 Direct Digital Synthesis Arduino Library

AD9959 is a chip from Analog Devices for direct digital sythesis
of radio frequency signals.  With four channels and a 500MHz core
frequency, it can coordinate multi-channel sweeps over frequency,
amplitude or phase and supports high-rate modulation.  This is a
template class for controlling the AD9959.

![AD9959 Example](examples/AD9959Sweep/AD9959Sweep.jpg)

## Hardware connection

The AD9959 has both 3v3 and 1v8 supplies. The digital interface is
3v3, so if you're using 5v, you need level shifters. This library
expects to use SPI in three-wire mode (CLK = SCLK, MISO = SDIO_2,
MOSI = SDIO_0).  SDIO_1 can float, but SDIO_3 should be pulled down.
The chip also needs RESET, a chip enable, I/O_UPDATE and four Profile
pins. This library expects you to control the profile pins if you
want to start and stop sweeps.

The internal PLL clock multiplier defaults to 20, which produces
500MHz from a 25MHz crystal. You can call setClock() to change
this if needed, or to apply a frequency calibration constant.

This library does not support configuring modulation or the power-down
modes. Ramp-up and ramp-down (amplitude ramping, available when
using frequency or phase modulation) is also not available.

## Setup Functions

Instantiate the AD9959 template with the appropriate parameters.
You must provide pin numbers for Chip Enable, Reset, and I/O Update.

The reference_freq parameter provides your crystal frequency.
The DDS core then runs at that frequency times the PLL multiplier,
which defaults to the maximum of 20.

The SPIRate parameter sets the SPI bit-rate. This library uses the
Arduino standard hardware SPI device.

    class MyAD9959 : public AD9959<
        2,              // Reset pin (active = high)
        3,              // Chip Enable (active = low)
        3,              // I/O_UPDATE: Apply config changes (pulse high)
        25000000        // 25MHz crystal (optional)
    > {};

    MyAD9959	dds;

The AD9959 reset is performed at the time this constructor is run.
Alternatively, you can reset the chip at any time later:

    dds.reset();

## Changing the DDS core frequency

Configure the PLL multiplier or apply a core frequency calibration
by calling setClock().
The default core frequency is 20 times the reference frequency.
You can set the multiplier as low as 4. Any other value disables the multiplier.

The calibration parameter supports frequency calibration.
Set it first to the default 10000000, program the chip to emit
a 10MHz signal, and measure the actual frequency. The measured
frequency is your new calibration value.

    setClock(int mult = 20, uint32_t calibration = 10000000)

    dds.setClock(4, 9987654);

After reset or changing the PLL multiplier, the core clock will take
up to 1 millisecond to stabilise. This library does not insert that
delay, so you can use that time to initialise other things.

## Setting the frequency, amplitude and phase

The frequency divider for a given frequency is obtained using
frequencyDivider().  Because this conversion uses a slow 64-bit
divide, you might save the result to use again.

    uint32_t	div;
    div = dds.frequencyDivider(455000);

The functions setFrequency(), setDivider(), setAmplitude() and
setPhase() prepare the signal(s) to generate:

    dds.setFrequency(MyAD9959::Channel2, 7140000UL);	// shorthand for:
    dds.setDivider(MyAD9959::Channel2, frequencyDivider(7140000UL));	// 7.14MHz
    dds.setAmplitude(MyAD9959::Channel2, 1023);		// Maximum amplitude value
    dds.setPhase(MyAD9959::Channel2, 16383);		// Maximum phase value (same as -1)

You can prepare multiple channels at the same time by OR-ing the
channel numbers together.  After preparing the signals, activate
all changes at the same time:

    dds.update();

## Sweeps

To make a sweep, you must configure the starting signals as above,
then either the destination frequency, amplitude or phase, and
sweep mode.

There are two sweep modes; follow (default) and no-dwell. In follow mode,
the sweep direction always follows the profile pin; sweeping up when the
signal is high, sweeping down when the signal is low, and stopping when
it reaches the defined destination. In no-dwell mode, setting the profile
pin high causes a rising sweep to start, hit the top, then jump back to
the starting value and stay there until the next positive edge.

    void sweepFrequency(ChannelNum chan, uint32_t freq, bool follow = true);
    ... etc

    dds.sweepFrequency(MyAD9959::Channel0|MyAD9959::Channel1, 8000000);	// Target frequency
    dds.sweepDivider(MyAD9959::Channel0|MyAD9959::Channel1, div, false);	// Target frequency divider
    dds.sweepAmplitude(MyAD9959::Channel0|MyAD9959::Channel1, 512);	// Target amplitude (half)
    dds.sweepPhase(MyAD9959::Channel0|MyAD9959::Channel1, 8192);	// Target phase (180 degrees)

Next, set up and down sweep rates by providing the step size and
the step rate.  A frequency step is a change in the divider value,
so you'll need to use frequencyDivider() to get the exact values.
The step rate is 8 bit unsigned, expressing a multiple of 4 core
clock cycles. With a 500MHz core clock, the shortest step is 8ns,
a step of 125 gets you 1us steps, and the longest step is 2.048us.

    void sweepRates(ChannelNum chan, uint32_t increment, uint8_t up_rate, uint32_t decrement, uint8_t down_rate);

    // Sweep up at increments of 100 each 1us, and down 1000 each 2us:
    dds.sweepRates(MyAD9959::Channel0|MyAD9959::Channel1, 100, 125, 1000, 250);

After setting up one or more channels' sweep parameters, start the
sweep by toggling the configured profile pin(s). If you toggle the
pins together, or wire two profile pins to the same output, they'll
start in sync.

You can change the sweep parameters during a sweep.  Unfortunately
there's no way to read back the sweep progress, so you must track
the time since you started the sweep to know what it's up to and
when it's finished.

## Reading internal registers

You can read back the value of any of the registers. Beware that
reading any of the (duplicated) channel registers requires first
selecting just one channel:

    dds.setChannels(MyAD9959::Channel0);
    uint32_t value = dds.read(MyAD9959::CFTW);	// Read the current divider
