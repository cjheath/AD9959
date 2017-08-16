# AD9959 Direct Digital Synthesis Arduino Library

AD9959 is a chip from Analog Devices for direct digital sythesis of radio frequency signals.
With four channels and a 500MHz core frequency, it can coordinate multi-channel sweeps over
frequency, amplitude or phase and supports high-rate modulation.
This is a template class for controlling the AD9959.

![AD9959 Example](examples/AD9959Sweep/AD9959Sweep.jpg)

## Hardware connection

The AD9959 has both 3v3 and 1v8 supplies. The digital interface is 3v3,
so if you're using 5v, you need level shifters. This library expects
to use SPI in three-wire mode (CLK = SCLK, MISO = SDIO_2, MOSI = SDIO_0).
SDIO_1 can float, but SDIO_3 should be pulled down. The chip also needs
RESET, a chip select, I/O_UPDATE and four Profile pins. This library
expects you to control the profile pins to start and stop sweeps.

The internal PLL clock multiplier is set to 20, which produces 500MHz from
a 25MHz crystal. You'll have to edit the code to change this if needed.

This library does not allow configuring the power-down modes.

## Setup Functions

Instantiate the AD9959 template with the appropriate parameters.
You must provide pin numbers for Chip Enable, Reset, I/O Update, and Sweep start.

The Calibration parameter to the template supports frequency calibration.
Set it to 10000000, program the chip to emit a 10MHz signal, and measure the
actual frequency.  Then change the calibration parameter to the measured frequency.

The SPIRate parameter sets the SPI bit-rate.

You can also provide SPI pins, but the library uses the hardware SPI anyway.
This is a problem with the SPI library, which should support consistent access
to both hardware and software implementations of SPI.

    class MyAD9959 : public AD9959<
        2,              // Reset pin (active = high)
        3,              // Data Load pin (active = pulse high)
        3,              // I/O_UPDATE: Apply config changes
        10000000        // Use your actual frequency when set to 10MHz
    > {};

    MyAD9959	dds;

The AD9959 reset will be applied at the time this constructor is run.
Alternatively, you can reset the chip at any time later:

    dds.reset();

## Sections below here are not yet implemented; this API is proposed.

## Setting the frequency, amplitude and phase

The functions setFrequency(), setAmplitude() and setPhase() prepare the signal(s) to generate:

    dds.setFrequency(MyAD9959::Channel2, 7140000UL);	// 7.14MHz
    dds.setAmplitude(MyAD9959::Channel2, 1023);		// Maximum amplitude value
    dds.setPhase(MyAD9959::Channel2, 16383);		// Maximum phase value

You can prepare multiple channels at the same time by ORing the channel numbers together,
or separately. After preparing the signals, activate all changes at the same time:

    dds.update();

If you need to know the frequency divider for a given frequency
(e.g. for use in sweeps), use the function frequencyDivider():

    uint32_t	div;
    div = dds.frequencyDivider(455000);

## Sweeps

To make a sweep, you must configure the starting signals as above,
then either the destination frequency, amplitude or phase:

    dds.sweepFrequency(MyAD9959::Channel0|MyAD9959::Channel1, 8000000);	// Target frequency
    dds.sweepAmplitude(MyAD9959::Channel0|MyAD9959::Channel1, 512);	// Target amplitude (half)
    dds.sweepPhase(MyAD9959::Channel0|MyAD9959::Channel1, 8192);	// Target phase (180 degrees)

Next, set up and down sweep rates by providing the step size and the step rate.
A frequency step size is converted from a frequency value into a divider value, so
it's not exact (32 bit resolution) - use frequencyDivider() to get the exact values.
The step rate is 8 bit unsigned, expressing a multiple of 4 core clock cycles
(125MHz if 500MHz core), so the shortest step is 8ns, the longest step is 2.048us.
You can change the sweep parameters during a sweep.

    dds.sweepUpRate(MyAD9959::Channel0|MyAD9959::Channel1, 100, 1000);	// Set the sweep up rate
    dds.sweepDownRate(MyAD9959::Channel0|MyAD9959::Channel1, 100, 1000);// Set the sweep down rate

There are two sweep modes; follow (default) and no-dwell. In follow mode,
the sweep direction always follows the profile pin; sweeping up when the
signal is high, sweeping down when the signal is low, and stopping when
it reaches the defined destination. In no-dwell mode, setting the profile
pin high causes a rising sweep to start, hit the top, then jump back to
the starting value and stay there until the next positive edge.

    dds.sweepFollow(bool = true);	// sweep in the direction indicated by the profile pin

After setting up one or more channels' sweep parameters, start the sweep by
toggling the configured profile pin(s). If you toggle the pins together, or
wire two profile pins to the same output, they'll start in sync.

<!---
You can read out the sweep accumulator register at any time during a sweep.
Combined with the frequencyDivider() function and the ability to change
sweep parameters on-the-fly, you can do some pretty tricky things like
near-log sweeps instead of linear. Or just track the sweep progress (but
the SPI communication might increase noise levels).

    acc = dds.sweepProgress(MyAD9959::Channel0|MyAD9959::Channel1);	// Read the sweep accumulator value
--->
