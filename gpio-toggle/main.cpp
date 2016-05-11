#include <xpcc/architecture/platform.hpp>

using namespace Board;

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} rgb_t;

static constexpr uint8_t NUM_LEDS = 10;

rgb_t leds[NUM_LEDS];

int
main()
{
	initialize();
	GpioOutputB9::setOutput();
	GpioOutputB9::set(false);
	LedUp::reset();

	for (uint8_t i = 0; i < NUM_LEDS; i++) {
		leds[i].r = 0xFF;
		leds[i].g = 0;
		leds[i].b = 0;
	}

	while (1) {
		/*
		for (uint8_t i = 0; i < NUM_LEDS; i++) {
			uint8_t r = leds[i].r;
			for (uint8_t j = 0; j < 8; j++) {
				if (r & (1 << j)) { // Bit is 1
					GpioOutputB9::set(true);
					xpcc::delayNanoseconds(900);
					GpioOutputB9::set(false);
					xpcc::delayNanoseconds(350);
				}
				else { // Bit is 1
					GpioOutputB9::set(true);
					xpcc::delayNanoseconds(350);
					GpioOutputB9::set(false);
					xpcc::delayNanoseconds(900);
				}
			}
			uint8_t g = leds[i].g;
			for (uint8_t j = 0; j < 8; j++) {
				if (g & (1 << j)) { // Bit is 1
					GpioOutputB9::set(true);
					xpcc::delayNanoseconds(900);
					GpioOutputB9::set(false);
					xpcc::delayNanoseconds(350);
				}
				else { // Bit is 1
					GpioOutputB9::set(true);
					xpcc::delayNanoseconds(350);
					GpioOutputB9::set(false);
					xpcc::delayNanoseconds(900);
				}
			}
			uint8_t b = leds[i].b;
			for (uint8_t j = 0; j < 8; j++) {
				if (b & (1 << j)) { // Bit is 1
					GpioOutputB9::set(true);
					xpcc::delayNanoseconds(900);
					GpioOutputB9::set(false);
					xpcc::delayNanoseconds(350);
				}
				else { // Bit is 1
					GpioOutputB9::set(true);
					xpcc::delayNanoseconds(350);
					GpioOutputB9::set(false);
					xpcc::delayNanoseconds(900);
				}
			}
		}

		GpioOutputB9::set(false);
		xpcc::delayMicroseconds(100);
*/

		for (uint16_t i = 0; i < (3*8*12); i++) {
			GpioOutputB9::set();
			GpioOutputB9::set();

			//xpcc::delayNanoseconds(10);
			GpioOutputB9::reset();
			xpcc::delayNanoseconds(900);
		}

		LedUp::toggle();
		xpcc::delayMilliseconds(20);
	}
}
