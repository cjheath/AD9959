#define ARDUINO 10600

#include	<stdio.h>

typedef enum { LOW = 0, HIGH = 1} PinValue;
inline void digitalWrite(int pin, bool value) {}

typedef enum { INPUT = 0, OUTPUT = 1} PinMode;
inline void pinMode(int pin, PinMode mode) {}

struct Serial {
	void	nl() { printf("\n"); }
	void	print(const char* s) { printf("%s", s); }
	void	println(const char* s) { print(s); nl(); }
	void	print(uint32_t i) { printf("%d", i); }
	void	println(uint32_t i) { print(i); nl(); }
} Serial;
