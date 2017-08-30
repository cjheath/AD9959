/*
 * Test program for divisor calculation using reciprocal division.
 */
#include	<inttypes.h>
#include	<stdlib.h>
#include	<string.h>
#include	<math.h>

#include	"AD9959.h"

#define	INCR		0.000005	// The smaller this number, the more test data points
#define	MAXERR		0.08
#define	MAX_FREQ	(500*1000*1000)

bool	verbose = false;

class MyAD9959
: public AD9959<
	2,		// ResetPin
	3,		// ChipEnablePin
	4		// UpdatePin
	// 25000000 (reference_frequency)
	// SPIRate, SPIClkPin, SPIMISOPin, SPIMOSIPin
> {};

void test_dividers(MyAD9959& dds)
{
	int		count = 0;
	int		low = 0;
	int		high = 0;
	uint32_t	last = 1;
	uint32_t	freq = 0;
	int		decade = 2;
	int		pow10 = 10;
	double		two32 = pow(2, 32);
	double		sum_of_squares_gen = 0;
	double		sum_of_squares_acc = 0;
	double		sum_of_squares_high = 0;
	double		sum_of_squares_low = 0;
	uint32_t	core_clock = MAX_FREQ;

#if	defined(CLOCK_MULTIPLIER)
	// Test with a reduced clock rate
	dds.setClock(CLOCK_MULTIPLIER);	// Half the core_clock frequency
	core_clock /= (20/CLOCK_MULTIPLIER);
#endif

	for (double freqr = 10; freqr <= core_clock/2; freqr *= (1+INCR))
	{
		freq = floor(freqr+0.5);
		if (freq == last)
			continue;
		last = freq;

		if (freq > pow10*10)
		{
			decade++;
			pow10 *= 10;
			printf("Progress to %d: %d frequencies tested, %d low, %d high, %d bad\n", pow10, count, low, high, count-low-high);
		}

		uint32_t	divisor = dds.frequencyDelta(freq);
		uint32_t	accurate = floor(two32*freq/core_clock + 0.500);
		double		fgen = 1ULL*core_clock*divisor/two32;
		double		facc = 1ULL*core_clock*accurate/two32;
		double		epsilon_gen = fgen-freq; // difference between desired and generated, reciprocal method
		double		epsilon_acc = facc-freq; // difference between desired and generated, normal method
		sum_of_squares_gen += epsilon_gen*epsilon_gen;
		sum_of_squares_acc += epsilon_acc*epsilon_acc;
		bool		bad = 0;
		if (fgen > freq)
		{
			sum_of_squares_high += epsilon_gen*epsilon_gen;
			if (fgen < freq+MAXERR)
				high++;
			else
				bad = 1;
		}
		if (fgen < freq)
		{
			sum_of_squares_low += epsilon_gen*epsilon_gen;
			if (fgen > freq-MAXERR)
				low++;
			else
				bad = 1;
		}

		// printf("Wanted %u, Gen %0.4f Acc %0.4f Epsilon gen %g acc %g\n", freq, fgen, facc, epsilon_gen, epsilon_acc);

		if (verbose)
			printf("%u -> %u for %.4f ", freq, divisor, fgen);

		if (divisor == accurate)
		{
			if (verbose)
				printf("good\n");
		}
		else
		{
			if (verbose)
				printf("expected %u (%s)\n", accurate, bad ? "bad" : "acceptable");
		}
		count++;
	}
	printf("%d frequencies tested, %d low, %d high, %d bad\n", count, low, high, count-low-high);
	printf(
		"Frequency Standard Deviation using reciprocal method %gHz, accurate %gHz\n",
		sqrt(sum_of_squares_gen/count),
		sqrt(sum_of_squares_acc/count)
	);
	printf("Upward deviation %.5g\n", sqrt(sum_of_squares_high/count));
	printf("Downward deviation %.5g\n", sqrt(sum_of_squares_low/count));
}

int
main(int argc, const char** argv)
{
	MyAD9959	dds;

	if (--argc > 0 && strcmp(*++argv, "-v") == 0)
		verbose = 1;
	test_dividers(dds);
}
