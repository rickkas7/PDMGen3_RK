
#include "PDMGen3_RK.h"

// #include "pinmap_hal.h"


PDMGen3 *PDMGen3::instance = NULL;

PDMGen3::PDMGen3(pin_t clkPin, pin_t datPin, int16_t *samples, size_t samplesPerBuf) : clkPin(clkPin), datPin(datPin), samples(samples), samplesPerBuf(samplesPerBuf) {
	instance = this;
}

PDMGen3::~PDMGen3() {

}

nrfx_err_t PDMGen3::init() {
	Hal_Pin_Info *pinMap = HAL_Pin_Map();

	pinMode(clkPin, OUTPUT);
	pinMode(datPin, INPUT);

	attachInterruptDirect(PDM_IRQn, nrfx_pdm_irq_handler, false);

	uint8_t nrfClkPin = (uint8_t)NRF_GPIO_PIN_MAP(pinMap[clkPin].gpio_port, pinMap[clkPin].gpio_pin);
	uint8_t nrfDatPin = (uint8_t)NRF_GPIO_PIN_MAP(pinMap[datPin].gpio_port, pinMap[datPin].gpio_pin);

	// Start with default vales
	nrfx_pdm_config_t config = NRFX_PDM_DEFAULT_CONFIG(nrfClkPin, nrfDatPin);

	// Override with everything we have local copies of
	config.mode = mode;
	config.clock_freq = freq;
	config.edge = edge;
	config.gain_l = gainL;
	config.gain_r = gainR;

	// Initialize!
	nrfx_err_t err = nrfx_pdm_init(&config, dataHandlerStatic);

	return err;
}

void PDMGen3::uninit() {
	availableSamples = NULL;

	nrfx_pdm_uninit();

	pinMode(clkPin, INPUT);
}

nrfx_err_t PDMGen3::start() {
	useBufferA = true;
	availableSamples = NULL;

	nrfx_err_t err = nrfx_pdm_start();

	return err;
}

nrfx_err_t PDMGen3::stop() {
	availableSamples = NULL;

	nrfx_err_t err = nrfx_pdm_stop();

	return err;
}

PDMGen3 &PDMGen3::withGainDb(float gain) {
	if (gain < -20.0) {
		gain = -20.0;
	}
	if (gain > 20.0) {
		gain = 20.0;
	}

	int16_t halfSteps = (int16_t)(gain * 2);

	// nRF52 values are from:
	// -20 dB = 0x00
	// 0 dB   = 0x28 = 40
	// +20 dB = 0x50 = 80
	return withGain(halfSteps + 40, halfSteps + 40);
}

void PDMGen3::dataHandler(nrfx_pdm_evt_t const * const pEvent) {
	/*
 	bool             buffer_requested;  ///< Buffer request flag.
    int16_t *        buffer_released;   ///< Pointer to the released buffer. Can be NULL.
    nrfx_pdm_error_t error;             ///< Error type.
	 */

	if (pEvent->buffer_released) {
		// Adjust samples here
		int16_t *src = (int16_t *)pEvent->buffer_released;

		if (outputSize == OutputSize::UNSIGNED_8) {
			uint8_t *dst = (uint8_t *)src;

			// Scale the 16-bit signed values to an appropriate range for unsigned 8-bit values
			int16_t div = (int16_t)(1 << (size_t) range);

			for(size_t ii = 0; ii < samplesPerBuf; ii++) {
				int16_t val = src[ii] / div;

				// Clip to signed 8-bit
				if (val < -128) {
					val = -128;
				}
				if (val > 127) {
					val = 127;
				}

				// Add 128 to make unsigned 8-bit (offset)
				dst[ii] = (uint8_t) (val + 128);
			}

		}
		else
		if (outputSize == OutputSize::SIGNED_16) {
			int32_t mult = (int32_t)(1 << (8 - (size_t) range));
			for(size_t ii = 0; ii < samplesPerBuf; ii++) {
				// Scale to signed 16 bit range
				int32_t val = (int32_t)src[ii] * mult;

				// Clip to signed 16-bit
				if (val < -32767) {
					val = -32767;
				}
				if (val > 32768) {
					val = 32868;
				}

				src[ii] = (int16_t) val;
			}
		}
		availableSamples = src;

		if (interruptCallback) {
			interruptCallback(src, samplesPerBuf);
		}
	}

	if (pEvent->buffer_requested) {
		if (useBufferA) {
			nrfx_pdm_buffer_set(samples, samplesPerBuf);
		}
		else {
			nrfx_pdm_buffer_set(&samples[samplesPerBuf], samplesPerBuf);
		}
		useBufferA = !useBufferA;
	}
}



// [static]
void PDMGen3::dataHandlerStatic(nrfx_pdm_evt_t const * const pEvent) {
	if (instance) {
		instance->dataHandler(pEvent);
	}
}



