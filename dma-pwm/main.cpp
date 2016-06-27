#include <xpcc/architecture/platform.hpp>
#include <xpcc/architecture.hpp>
#include <xpcc/debug/logger.hpp>

#include <xpcc/architecture/platform/driver/dma/stm32/type_ids.hpp>

// ----------------------------------------------------------------------------
// Set the log level
#undef	XPCC_LOG_LEVEL
#define	XPCC_LOG_LEVEL xpcc::log::INFO

// Create an IODeviceWrapper around the Uart Peripheral we want to use
xpcc::IODeviceWrapper< Usart1, xpcc::IOBuffer::BlockIfFull > loggerDevice;

// Set all four logger streams to use the UART
xpcc::log::Logger xpcc::log::debug(loggerDevice);
xpcc::log::Logger xpcc::log::info(loggerDevice);
xpcc::log::Logger xpcc::log::warning(loggerDevice);
xpcc::log::Logger xpcc::log::error(loggerDevice);

using namespace Board;
using namespace xpcc::stm32;

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
} rgb_t;

static constexpr uint8_t numLeds = 5;
static constexpr uint16_t numDmaTransactions = numLeds * 3 * 8 + 50;

rgb_t* leds;

uint16_t* timerValues;


/* Timer2::setPeriod(cast<integer>1.25, false); // 1.25µS needed
 *
 * setPeriod(uint32_t microseconds, bool autoApply = true):
 * This will be inaccurate for non-smooth frequencies (last six digits unequal to zero)
 * uint32_t cycles = microseconds * (SystemClock::Timer2 / 1000000UL);
 * uint16_t prescaler = (cycles + 65535) / 65536;	// always round up
 * uint16_t overflow = cycles / prescaler;
 *
 * overflow = overflow - 1;	// e.g. 36000 cycles are from 0 to 35999
 */
static constexpr uint32_t cycles = 1.25 * (systemClock::Frequency / 1000000UL);
static constexpr uint16_t prescaler = (cycles + 65535) / 65536;	// always round up
static constexpr uint16_t overflow = (cycles / prescaler) - 1;	// e.g. 36000 cycles are from 0 to 35999

static constexpr Timer2::Value compareValue0 = overflow * (350.0 / 1250.0);
static constexpr Timer2::Value compareValue1 = overflow * (900.0 / 1250.0);

void ledsToTimerValues(rgb_t* leds, uint16_t* timerValues, uint32_t numLeds);


inline uint32_t isr() {
	return DMA1->ISR;
}
inline uint32_t cndtr() {
	return DMA1_Channel3->CNDTR;
}
inline uint32_t tim2_ccr2() {
	return TIM2->CCR2;
}

int
main()
{
	initialize();
	LedUp::set();
	LedDown::set();

	// Enable USART 1
	GpioOutputA9::connect(Usart1::Tx);
	GpioInputA10::connect(Usart1::Rx, Gpio::InputType::PullUp);
	Usart1::initialize<Board::systemClock, Usart1::B115200>(12);

	leds = new (xpcc::MemoryDefault) rgb_t[numLeds];
	timerValues = new (xpcc::MemoryDMA | xpcc::MemoryFastData) uint16_t[numDmaTransactions];

	for (uint32_t i = 0; i < numLeds; i++) {
		leds[i].r = 0b01010101;
		leds[i].g = 0b01010101;
		leds[i].b = 0b01010101;
	}
	for (uint32_t i = 0; i < numDmaTransactions; i++) {
		timerValues[i] = 0;
	}

	XPCC_LOG_INFO << "numLeds=" << numLeds << xpcc::endl;
	XPCC_LOG_INFO << "numDmaTransactions=" << numDmaTransactions << xpcc::endl;

	// Pwm output pin
	GpioOutputA1::setOutput(xpcc::Gpio::Low);
	GpioOutputA1::connect(Timer2::Channel2);

	// enable and setup timer TIM2
	Timer2::enable();
	Timer2::setMode(Timer2::Mode::UpCounter);
	Timer2::setOverflow(overflow);
	Timer2::setPrescaler(prescaler);
	Timer2::configureOutputChannel(2, Timer2::OutputCompareMode::Pwm, 42);
	//Timer2::enableDmaRequest(Timer2::DmaRequestEnable::CaptureCompare2);
	//TIM2->CR1 |= TIM_CR1_ARPE; //  Auto-reload preload enable
	//TIM2->CCMR1 |= TIM_CCMR1_OC1PE; // Output compare 1 preload enable
	//Timer2::applyAndReset();


	XPCC_LOG_INFO << "TIM2->CR1=" << ((uint32_t)TIM2->CR1) << xpcc::endl;
	XPCC_LOG_INFO << "TIM2->CCMR1=" << ((uint32_t)TIM2->CCMR1) << xpcc::endl;

	//Timer2::start();

	// enable and setup DMA
	Dma1::enable();
	Dma1::Stream3::stop();


	while (1) {
		ledsToTimerValues(leds, timerValues, numLeds);

		//DMA1->IFCR = 0b0000011100000000; // Reset HTIF3, TCIF3 and GIF3
		Dma1::Stream3::setPeripheralDestination(reinterpret_cast<uint16_t*>(const_cast<uint32_t*>(&(TIM2->CCR2))));
		Dma1::Stream3::setMemorySource(static_cast<uint16_t*>(timerValues));
		Dma1::Stream3::configure(numDmaTransactions-1, Dma1::Stream3::Priority::VeryHigh, Dma1::Stream3::CircularMode::Disabled);
		Dma1::Stream3::start();

		Timer2::enableDmaRequest(Timer2::DmaRequestEnable::CaptureCompare2);
		Timer2::setCompareValue(2, timerValues[0]);
		Timer2::applyAndReset();

		XPCC_LOG_INFO << "!!!! DMA stream3 restart !!!!" << xpcc::endl;

		XPCC_LOG_INFO << "DMA1->CNDTR=" << cndtr() << xpcc::endl;
		XPCC_LOG_INFO << "DMA1->ISR=" << isr() << xpcc::endl;

		Timer2::start();


		while (!Dma1::Stream3::isFinished()) {
			//XPCC_LOG_INFO << "DMA1->CNDTR=" << cndtr() << xpcc::endl;
			//XPCC_LOG_INFO << "TIM2->CCR2=" << tim2_ccr2() << xpcc::endl;
			XPCC_LOG_INFO << "DMA1->CNDTR=" << cndtr() << xpcc::endl;
			LedRight::toggle();
		}
		XPCC_LOG_INFO << "DMA1->CNDTR=" << cndtr() << xpcc::endl;
		XPCC_LOG_INFO << "DMA1->ISR=" << isr() << xpcc::endl;
		XPCC_LOG_INFO << "!!!! DMA stream3 finished !!!!" << xpcc::endl;

		Timer2::pause();
		Dma1::Stream3::stop();

		XPCC_LOG_INFO << "DMA1->CNDTR=" << cndtr() << xpcc::endl;
		XPCC_LOG_INFO << "DMA1->ISR=" << isr() << xpcc::endl;

		xpcc::delayMilliseconds(500);
		LedDown::toggle();
	}
}

void ledsToTimerValues(rgb_t* leds, uint16_t* timerValues, uint32_t numLeds) {
	XPCC_LOG_INFO << "ledsToTimerValues() start" << xpcc::endl;
	uint16_t timerValuesIndex = 0;
	for (uint8_t i = 0; i < numLeds; i++) {
		// 24 Datenbits (G8:R8:B8), je Bit 1 TimerValue
		uint8_t b;
		for (uint8_t k = 0; k < 3; k++) {
			switch(k) {
			case 0:
				b = leds[i].g;
				break;
			case 1:
				b = leds[i].r;
				break;
			case 2:
				b = leds[i].b;
				break;
			}
			for (uint8_t j = 0; j < 8; j++) {
				if (b & (1 << j)) {
					timerValues[timerValuesIndex++] = compareValue1;
				}
				else {
					timerValues[timerValuesIndex++] = compareValue0;
				}
			}
		}
	}
	while (timerValuesIndex < numDmaTransactions) {
		timerValues[timerValuesIndex++] = 0;
	}
	/*XPCC_LOG_INFO << "numDmaTransactions=" << numDmaTransactions << xpcc::endl;
	for (uint16_t i = 0; i < numDmaTransactions; i++) {
		XPCC_LOG_INFO << "compareValue=" << timerValues[i] << xpcc::endl;
	}*/
	XPCC_LOG_INFO << "ledsToTimerValues() end" << xpcc::endl;
}
