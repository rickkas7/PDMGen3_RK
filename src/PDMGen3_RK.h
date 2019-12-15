#ifndef __PDMGEN3_RK_H
#define __PDMGEN3_RK_H

#include "Particle.h"

#include "nrfx_pdm.h"

/**
 * @brief Class for using the hardware PDM (pulse-density modulation) audio decoder on nRF52 devices
 *
 * This only works on Gen 3 device (Argon, Boron, Xenon). Gen 2 devices include a STM32F205 MCU which
 * does not include PDM support.
 *
 * You typically create one PDMGen3 object as a global variable in your application.
 */
class PDMGen3 {
public:
	/**
	 * @brief Configuration for the size of the output data
	 */
	enum class OutputSize {
		UNSIGNED_8,	 	//!< Output unsigned 8-bit values (adjusted by PDMRange)
		SIGNED_16,		//!< Output signed 16-bit values (adjusted by PDMRange) (default)
		RAW_SIGNED_16	//!< Output values as signed 16-bit values as returned by nRF52 (unadjusted)
	};

	/**
	 * @brief Adjustment value so samples fill most of an 8 or 16 bit output size
	 */
	enum class Range {
		RANGE_128 = 0, 	//!< From -128 to 127 (8 bits)
		RANGE_256, 		//!< From -256 to 255 (9 bits)
		RANGE_512, 		//!< From -512 to 511 (10 bits)
		RANGE_1024, 	//!< From -1024 to 1023 (11 bits)
		RANGE_2048, 	//!< From -2048 to 2047 (12 bits) (default)
		RANGE_4096, 	//!< From -4096 to 4095 (13 bits)
		RANGE_8192, 	//!< From -8192 to 8191 (14 bits)
		RANGE_16384, 	//!< From -16384 to 16383 (15 bits)
		RANGE_32768, 	//!< From -32768 to 32767 (16 bits) (same as raw mode)
	};


	/**
	 * @brief Allocate a PDMGen3 object for using the hardware PDM decoder
	 *
	 * @param clkPin Clock pin. Use any available GPIO: A0, A1, ..., D0, D1, ... even pins like RX, TX,
	 * etc. if you are not using that interface. Will be set as OUTPUT.
	 *
	 * @param datPin Data pin. Use any available GPIO. Will be set as INPUT.
	 *
	 * @param samples Pointer to a buffer to hold samples. Note: This must be 2 * samplesPerBuf
	 * int16_t values, and the number of bytes used will be 4 * samplesPerBuf because they're 16-bit
	 * samples.
	 *
	 * @param samplesPerBuf The number of int16_t (signed 16-bit) samples in each of the double
	 * buffers.
	 *
	 * You may want to use the PDMGen3Static class instead which statically allocates the buffers
	 * on the heap at compile time, eliminating the need to manually manage them.
	 */
	PDMGen3(pin_t clkPin, pin_t datPin, int16_t *samples, size_t samplesPerBuf);

	/**
	 * @brief Destructor. You typically allocate this as a global variable so it won't be deleted.
	 */
	virtual ~PDMGen3();

	/**
	 * @brief Initialize the PDM module.
	 *
	 * This is often done from setup(). You can defer it until you're ready to sample if desired,
	 * calling right before start().
	 */
	nrfx_err_t init();

	/**
	 * @brief Uniniialize the PDM module.
	 *
	 * You normally will just initialize it once and only start and stop it as necessary, however
	 * you can completely uninitialize it if desired. The clkPin will be reset to INPUT mode.
	 */
	void uninit();

	/**
	 * @brief Start sampling
	 */
	nrfx_err_t start();

	/**
	 * @brief Stop sampling
	 */
	nrfx_err_t stop();

	/**
	 * @brief Sets the gain in dB
	 *
	 * @param gainDb Gain in dB, from -20 (minimum) to +20 (maximum) in 0.5 dB steps
	 */
	PDMGen3 &withGainDb(float gainDb);

	/**
	 * @brief Sets the PDM gain using an rRF52 configuration value
	 *
	 * @param gainL The nRF52 gain value for the left or mono channel

	 * @param gainR The nRF52 gain value for the right channel
	 *
	 * - NRF_PDM_GAIN_MINIMUM (0x00)
	 * - NRF_PDM_GAIN_DEFAULT (0x28) 0 dB gain, default value
	 * - NRF_PDM_GAIN_MAXIMUM (0x50)
	 */
	PDMGen3 &withGain(nrf_pdm_gain_t gainL, nrf_pdm_gain_t gainR) { this->gainL = gainL; this->gainR = gainR; return *this; };


	/**
	 * @brief Set stereo mode. In stereo mode, each buffer will interleave L and R samples. Default is mono mode.
	 */
	PDMGen3 &withStereoMode() { return withMode(NRF_PDM_MODE_STEREO); };

	/**
	 * @brief Set mono mode (left channel only). The default is mono mode, so you normally don't need to set this.
	 *
	 * In mono mode, only the left channel is sampled, so every sample in the buffer will be from the left channel.
	 */
	PDMGen3 &withMonoMode() { return withMode(NRF_PDM_MODE_MONO); };


	/**
	 * @brief Sets a mode using a nrf_pdm_mode_t constant. Default is  NRF_PDM_MODE_MONOl
	 *
	 * In mono mode, only the left channel is sampled, so every sample in the buffer will be from the left channel.
	 */
	PDMGen3 &withMode(nrf_pdm_mode_t mode) { this->mode = mode; return *this; };


	/**
	 * @brief Sets the sampling frequency.  Default is 1.032 MHz, which works out to be a PCM sample rate of 16125 samples/sec.
	 *
	 * @param freq Sampling frequency: NRF_PDM_FREQ_1000K, NRF_PDM_FREQ_1032K (default), or NRF_PDM_FREQ_1067K
	 */
	PDMGen3 &withFreq(nrf_pdm_freq_t freq) { this->freq = freq; return *this; };

	/**
	 * @brief Sets the edge mode
	 *
	 * @param edge Edge mode. Whether left or mono channel is sample on falling CLK (default) or rising CLK
	 *
	 * - NRF_PDM_EDGE_LEFTFALLING Left (or mono) is sampled on falling edge of PDM_CLK (default)
     * - NRF_PDM_EDGE_LEFTRISING Left (or mono) is sampled on rising edge of PDM_CLK.
	 *
	 */
	PDMGen3 &withEdge(nrf_pdm_edge_t edge) { this->edge = edge; return *this; };

	/**
	 * @brief Sets the size of the output samples
	 *
	 * @param outputSize The output size enumeration
	 *
	 * - UNSIGNED_8     Output unsigned 8-bit values (adjusted by PDMRange)
	 * - SIGNED_16,	    Output signed 16-bit values (adjusted by PDMRange) (default)
	 * - RAW_SIGNED_16  Output values as signed 16-bit values as returned by nRF52 (unadjusted)
	 *
	 */
	PDMGen3 &withOutputSize(OutputSize outputSize) { this->outputSize = outputSize; return *this; };

	/**
	 * @brief Sets the range of the output samples
	 *
	 * @param range The range enumeration
	 *
	 * RANGE_128   From -128 to 127 (8 bits)
	 * RANGE_256   From -256 to 255 (9 bits)
	 * RANGE_512   From -512 to 511 (10 bits)
	 * RANGE_1024  From -1024 to 1023 (11 bits)
	 * RANGE_2048  From -2048 to 2047 (12 bits) (default)
	 * RANGE_4096  From -4096 to 4095 (13 bits)
	 * RANGE_8192  From -8192 to 8191 (14 bits)
	 * RANGE_16384 From -16384 to 16383 (15 bits)
	 * RANGE_32768 From -32768 to 32767 (16 bits) (same as raw mode)
	 *
	 */
	PDMGen3 &withRange(Range range) { this->range = range; return *this; };

	/**
	 * @brief Sets a callback function to be called at interrupt time
	 */
	PDMGen3 &withInterruptCallback(std::function<void(void *sampleBuf, size_t numSamples)> interruptCallback) { this->interruptCallback = interruptCallback; return *this; };

	/**
	 * @brief Call this to poll for available samples from loop()
	 *
	 * @return A point to a sample buffer or NULL if no samples are available
	 *
	 * Be sure to call doneWithSamples() if a non-NULL value is returned!
	 */
	void *getAvailableSamples() const { return (void *)availableSamples; };

	/**
	 * @brief Call when you are done using the samples returns from getAvailableSamples(). Required!
	 *
	 * This allows the buffer to be reused. If you don't call this, data will be corrupted.
	 */
	void doneWithSamples() { availableSamples = NULL; };

	/**
	 * @brief Returns the number of samples (not bytes) that was returned from getAvailableSamples()
	 */
	size_t getSamplesPerBuf() const { return samplesPerBuf; };


protected:
	/**
	 * @brief Used internally to handle notifications from the PDM peripheral
	 */
	void dataHandler(nrfx_pdm_evt_t const * const pEvent);

	/**
	 * @brief Used internally to handle notifications from the PDM peripheral (static function)
	 *
	 * As this relies on the singleton (instance), there can only be one instance of this class,
	 * however that's also the case because there is only one PDM peripheral on the nRF52.
	 */
	static void dataHandlerStatic(nrfx_pdm_evt_t const * const pEvent);

	pin_t clkPin;		//!< The pin used for the PDM clock (output)
	pin_t datPin;		//!< The pin used for the PDM data (input)
	int16_t *samples;	//!< Buffer to hold samples. There are 2x samples per buf because of double buffering
	size_t samplesPerBuf;//!< Number of samples per each of the double buffers
	nrf_pdm_gain_t gainL = NRF_PDM_GAIN_DEFAULT; 	//!< 0x28 = 0dB gain
	nrf_pdm_gain_t gainR = NRF_PDM_GAIN_DEFAULT; 	//!< 0x28 = 0dB gain
	nrf_pdm_mode_t mode = NRF_PDM_MODE_MONO;		//!< mono or stereo mode
	nrf_pdm_freq_t freq = NRF_PDM_FREQ_1032K;		//!< clock frequency
	nrf_pdm_edge_t edge = NRF_PDM_EDGE_LEFTFALLING; //!< clock edge configuration
	OutputSize outputSize = OutputSize::SIGNED_16;	//!< Output size (8 or 16 bits)
	Range range = Range::RANGE_2048;				//!< Range adjustment factor
	std::function<void(void *sampleBuf, size_t numSamples)> interruptCallback = NULL;	//!< buffer completion callback
	bool useBufferA = true;							//!< Which buffer we're reading from of the double buffers
	int16_t *availableSamples = NULL;				//!< Used to pass data to loop() for processing out of interrupt context

	/**
	 * @brief Singleton instance of this class
	 *
	 * Since there is only one PDM decoder on the nRF52 you must create only one instance of this class
	 * (typically as a global variable).
	 */
	static PDMGen3 *instance;
};


/**
 * @brief Construct a PDM decoder using a static buffer
 *
 * @param NUM_SAMPLES The number of samples to save. The number of bytes is 4x this value because each sample
 * is 2 bytes and there are double buffers. The number is must be large enough so you can process the data from
 * loop() without losing samples.
 */
template<size_t NUM_SAMPLES>
class PDMGen3Static : public PDMGen3 {
public:
	/**
	 * @brief Allocate a PDMGen3 object for using the hardware PDM decoder
	 *
	 * @param clkPin Clock pin. Use any available GPIO: A0, A1, ..., D0, D1, ... even pins like RX, TX,
	 * etc. if you are not using that interface. Will be set as OUTPUT.
	 *
	 * @param datPin Data pin. Use any available GPIO. Will be set as INPUT.
	 */
	PDMGen3Static(pin_t clkPin, pin_t datPin) : PDMGen3(clkPin, datPin, staticBuffer, NUM_SAMPLES) {};

	/**
	 * @brief Destructor. You typically allocate this as a global variable so it won't be deleted.
	 */
	virtual ~PDMGen3Static() {};

protected:
	int16_t staticBuffer[NUM_SAMPLES * 2];	//!< Buffer to hold samples
};

#endif /* __PDMGEN3_RK_H */
