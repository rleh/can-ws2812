#include <xpcc/architecture/platform.hpp>
#include <xpcc/architecture.hpp>
#include <xpcc/debug/logger.hpp>

#include <xpcc/architecture/platform/driver/dma/stm32/type_ids.hpp>

using namespace Board;
using namespace xpcc::stm32;

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} rgb_t;

static constexpr uint32_t numLeds = 10;
static constexpr uint32_t numDmaTransactions = numLeds * 3 * 16;

rgb_t leds[numLeds];

uint16_t timerValues[numDmaTransactions];

int
main()
{
	initialize();
	GpioOutputB9::setOutput();
	GpioOutputB9::set(false);
	LedUp::reset();

	for (uint8_t i = 0; i < numLeds; i++) {
		leds[i].r = 0xFF;
		leds[i].g = 0;
		leds[i].b = 0;
	}

	// enable and setup timer TIM2
	Timer17::enable();
	Timer17::setMode(Timer17::Mode::UpCounter);

	/* Timer17::setPeriod(cast<integer>1.25, false); // 1.25µS needed
	 *
	 * setPeriod(uint32_t microseconds, bool autoApply = true):
	 * This will be inaccurate for non-smooth frequencies (last six digits unequal to zero)
	 * uint32_t cycles = microseconds * (SystemClock::Timer17 / 1000000UL);
	 * uint16_t prescaler = (cycles + 65535) / 65536;	// always round up
	 * uint16_t overflow = cycles / prescaler;
	 *
	 * overflow = overflow - 1;	// e.g. 36000 cycles are from 0 to 35999
	 */
	uint32_t cycles = 1.25 * (systemClock::Frequency / 1000000UL);
	uint16_t prescaler = (cycles + 65535) / 65536;	// always round up
	uint16_t overflow = cycles / prescaler;
	overflow = overflow - 1;	// e.g. 36000 cycles are from 0 to 35999

	Timer17::Value compareValue0 = overflow / (350.0/1250.0);
	//Timer17::Value compareValue1 = overflow / (900.0/1250.0);

	Timer17::setOverflow(overflow);
	Timer17::setPrescaler(prescaler);

	// 	configureOutputChannel(uint32_t channel, OutputCompareMode mode, Value compareValue, PinState out = PinState::Enable);
	Timer17::configureOutputChannel(1, Timer17::OutputCompareMode::Pwm2, compareValue0);

	Timer17::enableDmaRequest(Timer17::DmaRequestEnable::CaptureCompare1);

	// Output pin?
	GpioOutputB9::connect(Timer17::Channel1);

	Timer17::applyAndReset();
	Timer17::start();


	// enable and setup DMA
	Dma1::enable();
	Dma1::Stream1::stop();
	Dma1::Stream1::configure(numDmaTransactions, Dma1::Stream1::Priority::VeryHigh, Dma1::Stream1::CircularMode::Disabled);
	Dma1::Stream1::setMemorySource(timerValues, Dma1::Stream1::MemoryIncrementMode::Increment);
	//Dma1::Stream1::setPeripheralDestination(static_cast<uint16_t*>&(Timer17::getCompareValue(Timer17::Channel1)), Dma1::Stream1::MemoryIncrementMode::Fixed);
	// xpcc-playground: Dma2::Stream0::setPeripheralSource(reinterpret_cast<uint16_t*>(const_cast<uint32_t*>(&(ADC1->DR))));
	Dma1::Stream1::setPeripheralDestination(reinterpret_cast<uint16_t*>(const_cast<uint32_t*>(&(TIM17->CCR1))), Dma1::Stream1::PeripheralIncrementMode::Fixed);
	//Dma1::Stream1::setPeripheralDestination(static_cast<uint16_t*>(&TIM17->CCR1), Dma1::Stream1::MemoryIncrementMode::Fixed);

	Dma1::Stream1::start();

	while (1) {
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
