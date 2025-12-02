#include <string>
#include <cstring>
#include <cmath>
#include <climits>
#include <csignal>
#include <complex>
#include <thread>
#include <mutex>
#ifndef RTLSDR
#include <iio.h>
#else
#include <rtl-sdr.h>
#endif
#include <unistd.h>
#define ALSA_PCM_NEW_HW_PARAMS_API
#include <alsa/asoundlib.h>
#include <iostream>

#define MHZ(x) (static_cast<long long>(x * 1000000. + .5))
#define GHZ(x) (static_cast<long long>(x * 1000000000. + .5))

#define AUDIO_SAMPLE_RATE 48000
#define AUDIO_SAMPLE_BITS 8

#define ALSA_PERIOD_DURATION_MS 50
#define ALSA_BUFFER_PERIODS 8

#define SAMPLE_FREQ MHZ(2.4)
#define LO_FREQ MHZ(107.5)
#define BANDWIDTH MHZ(0.2)
#ifndef RTLSDR
#define SAMPLES 64 * 1024
#else
#define SAMPLES 2 * 1024
#endif
#define SAMPLE_DEC 10

#define ARRAY_LENGTH(_x) (sizeof(_x) / sizeof((_x)[0]))

/* Low-pass filter, sampling_rate=2.4Mhz, cutoff_freq=120KHz */
const double lowpass_filter_coeffs[] = {
	-0.000223854182109267,
	-0.0006530797808749944,
	-0.0014941215383466283,
	-0.0028858221720252553,
	-0.004939404951176689,
	-0.007678669234947854,
	-0.010987644582794564,
	-0.014568399764957963,
	-0.017922319870453916,
	-0.020365792279901658,
	-0.021087878575380557,
	-0.019248725532304778,
	-0.014108795801050799,
	-0.005171436654364181,
	0.007686147286712376,
	0.024119811655711768,
	0.04328522669936062,
	0.0638792409270898,
	0.08425813671386242,
	0.10262004701468502,
	0.11722685655701257,
	0.12663226918946516,
	0.12987901196541202,
	0.12663226918946516,
	0.11722685655701257,
	0.10262004701468502,
	0.08425813671386242,
	0.0638792409270898,
	0.04328522669936062,
	0.024119811655711768,
	0.007686147286712376,
	-0.005171436654364181,
	-0.014108795801050799,
	-0.019248725532304778,
	-0.021087878575380557,
	-0.020365792279901658,
	-0.017922319870453916,
	-0.014568399764957963,
	-0.010987644582794564,
	-0.007678669234947854,
	-0.004939404951176689,
	-0.0028858221720252553,
	-0.0014941215383466283,
	-0.0006530797808749944,
	-0.000223854182109267,
};

/* Audio low-pass filter, sampling_rate=240KHz, cutoff_freq=15KHz */
const double audio_filter_coeffs[] {
	-0.00011443124614532072,
	-0.00017293152659996134,
	-0.0002537395679807246,
	-0.0002960101735381139,
	-0.00023955416024513902,
	-0.0000054285579679235705,
	0.0004964100334208889,
	0.0013538842757352097,
	0.0026349189106053485,
	0.004366935754078049,
	0.006517516169727653,
	0.00897952475800637,
	0.011566379161820748,
	0.01401887640845156,
	0.016028053953956276,
	0.017270887068045446,
	0.017457514231803247,
	0.016383318729148093,
	0.01397710066219987,
	0.010337169842383672,
	0.0057462823234805125,
	0.000658901918469194,
	-0.004341600521753473,
	-0.00861313183142416,
	-0.011545791615272886,
	-0.01266054361785775,
	-0.011700549440184652,
	-0.008698639621449431,
	-0.0040066848458185445,
	0.0017231736027468237,
	0.007608807120061617,
	0.012654244324838257,
	0.015898391911283113,
	0.016575938784746416,
	0.014265883702009357,
	0.009002241493722198,
	0.001325586224736748,
	-0.007739007276591902,
	-0.016781083407689842,
	-0.024172675639704693,
	-0.02828537119270258,
	-0.027727506101181958,
	-0.021567334814263428,
	-0.009507700677949066,
	0.008016566866991289,
	0.029837644916330886,
	0.05415546347475942,
	0.07873072365599945,
	0.1011464475013563,
	0.11910360136213313,
	0.13071022495045487,
	0.1347234472749074,
	0.13071022495045487,
	0.11910360136213313,
	0.1011464475013563,
	0.07873072365599945,
	0.05415546347475942,
	0.029837644916330886,
	0.008016566866991289,
	-0.009507700677949066,
	-0.021567334814263428,
	-0.027727506101181958,
	-0.02828537119270258,
	-0.024172675639704693,
	-0.016781083407689842,
	-0.007739007276591902,
	0.001325586224736748,
	0.009002241493722198,
	0.014265883702009357,
	0.016575938784746416,
	0.015898391911283113,
	0.012654244324838257,
	0.007608807120061617,
	0.0017231736027468237,
	-0.0040066848458185445,
	-0.008698639621449431,
	-0.011700549440184652,
	-0.01266054361785775,
	-0.011545791615272886,
	-0.00861313183142416,
	-0.004341600521753473,
	0.000658901918469194,
	0.0057462823234805125,
	0.010337169842383672,
	0.01397710066219987,
	0.016383318729148093,
	0.017457514231803247,
	0.017270887068045446,
	0.016028053953956276,
	0.01401887640845156,
	0.011566379161820748,
	0.00897952475800637,
	0.006517516169727653,
	0.004366935754078049,
	0.0026349189106053485,
	0.0013538842757352097,
	0.0004964100334208889,
	-0.0000054285579679235705,
	-0.00023955416024513902,
	-0.0002960101735381139,
	-0.0002537395679807246,
	-0.00017293152659996134,
	-0.00011443124614532072,
};

template <typename T>
class RingBuffer {
public:
	RingBuffer() = delete;
	RingBuffer(std::size_t length) : length(length), head(0), tail(0), data_length(0) {
		data = new T[length];
	}
	RingBuffer(const RingBuffer &source) {
		this->length = source.length;
		this->head = source.head;
		this->tail = source.tail;
		this->data_length = source.data_length;
		this->data = new T[length];
		std::memcpy(this->data, source.data, this->length);
	}
	RingBuffer &operator=(const RingBuffer &source) {
		delete [] data;
		this->length = source.length;
		this->head = source.head;
		this->tail = source.tail;
		this->data_length = source.data_length;
		this->data = new T[length];
		std::memcpy(this->data, source.data, this->length);
	}
	virtual ~RingBuffer() {
		delete [] data;
	}
	void Reset() {
		this->head = 0;
		this->tail = 0;
		this->data_length = 0;
	}
	T *GetData() const {
		return data;
	}
	T &operator[](std::size_t index) {
		if (index >= length)
			index = index % length;
		/* if (index >= length)
			throw std::out_of_range("Index out of range"); */
		return data[index];
	}
	T &GetHead() {
		return data[head];
	}
	T &GetTail() {
		return data[tail];
	}
	std::size_t GetHeadIndex() const {
		return head;
	}
	std::size_t GetTailIndex() const {
		return tail;
	}
	std::size_t GetLength() const {
		return length;
	}
	void MoveHeadIndex(std::size_t delta) {
		if (data_length + delta > length)
			throw std::out_of_range("Head index out of range");
		this->head = (head + delta) % length;
		this->data_length += delta;
	}
	void MoveTailIndex(std::size_t delta) {
		if (data_length < delta)
			throw std::out_of_range("Tail index out of range");
		this->tail = (tail + delta) % length;
		this->data_length -= delta;
	}
	std::size_t GetDataLength() const {
		return data_length;
	}
	std::size_t GetFree() const {
		return length - data_length;
	}
private:
	std::size_t length, head, tail, data_length;
	T *data;
};

template <typename T>
class Entity {
	public:
		Entity() = delete;
		Entity(std::size_t sink_length) : sink(sink_length) { }
		RingBuffer<T> &GetSinkBuffer() {
			return sink;
		}
	protected:
		RingBuffer<T> sink;
};

class SDR_RX : public Entity<std::complex<double>> {
public:
	SDR_RX() = delete;
	SDR_RX(const std::string &uri, long long freq) : Entity(SAMPLES << 1),
#ifndef RTLSDR
		ctx(nullptr), rx_dev(nullptr), rx_ch_i(nullptr), rx_ch_q(nullptr), rx_buf(nullptr)
#else
		dev(nullptr), temp(nullptr)
#endif
	{
#ifndef RTLSDR
		ctx = iio_create_context_from_uri(uri.c_str());
		if (!ctx)
			throw std::runtime_error("Cannot create context");

		unsigned int dev_cnt = iio_context_get_devices_count(ctx);
		if (!dev_cnt)
			throw std::runtime_error("No devices available");

		int chan_id = 0;
		iio_channel *chan = nullptr;

		iio_device *phy_dev = iio_context_find_device(ctx, "ad9361-phy");
		if (!phy_dev)
			goto close_ctx;

		chan = iio_device_find_channel(phy_dev, ("voltage" + std::to_string(chan_id)).c_str(), false);
		if (!chan)
			goto close_ctx;

		if (iio_channel_attr_write(chan, "rf_port_select", "A_BALANCED") < 0)
			goto close_ctx;

		if (iio_channel_attr_write_longlong(chan, "rf_bandwidth", BANDWIDTH) < 0)
			goto close_ctx;

		if (iio_channel_attr_write_longlong(chan, "sampling_frequency", SAMPLE_FREQ) < 0)
			goto close_ctx;

		if (iio_channel_attr_write(chan, "gain_control_mode", "fast_attack") < 0)
			goto close_ctx;

		chan = iio_device_find_channel(phy_dev, ("altvoltage" + std::to_string(0)).c_str(), true);
		if (!chan)
			goto close_ctx;

		if (iio_channel_attr_write_longlong(chan, "frequency", freq) < 0)
			goto close_ctx;

		rx_dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
		if (!phy_dev)
			goto close_ctx;

		rx_ch_i = iio_device_find_channel(rx_dev, ("voltage" + std::to_string(0)).c_str(), false);
		if (!rx_ch_i)
			rx_ch_i = iio_device_find_channel(rx_dev, ("altvoltage" + std::to_string(0)).c_str(), false);
		if (!rx_ch_i)
			goto close_ctx;

		rx_ch_q = iio_device_find_channel(rx_dev, ("voltage" + std::to_string(1)).c_str(), false);
		if (!rx_ch_q)
			rx_ch_q = iio_device_find_channel(rx_dev, ("altvoltage" + std::to_string(1)).c_str(), false);
		if (!rx_ch_q)
			goto disable_chan;

		iio_channel_enable(rx_ch_i);
		iio_channel_enable(rx_ch_q);

		rx_buf = iio_device_create_buffer(rx_dev, SAMPLES, false);
		if (!rx_buf)
			goto disable_chan;

		return;

disable_chan:
		iio_channel_disable(rx_ch_i);
		if (rx_ch_q)
			iio_channel_disable(rx_ch_q);

close_ctx:
		iio_context_destroy(ctx);
#else
		uint32_t dev_cnt = rtlsdr_get_device_count();
		if (!dev_cnt)
			throw std::runtime_error("No devices available");

		if (rtlsdr_open(&dev, std::stoul(uri)) < 0)
			throw std::runtime_error("Cannot open device");

		if (rtlsdr_set_testmode(dev, 0) < 0)
			goto close_dev;

		/* if (rtlsdr_set_dithering(dev, 1) < 0)
			goto close_dev; */

		if (rtlsdr_reset_buffer(dev) < 0)
			goto close_dev;

		if (rtlsdr_set_tuner_bandwidth(dev, BANDWIDTH) < 0)
			goto close_dev;

		if (rtlsdr_set_sample_rate(dev, SAMPLE_FREQ) < 0)
			goto close_dev;

		if (rtlsdr_set_center_freq(dev, freq) < 0)
			goto close_dev;

		if (rtlsdr_set_tuner_gain_mode(dev, 0) < 0)
			goto close_dev;

		temp = new uint8_t[SAMPLES << 1];

		return;

close_dev:
		rtlsdr_close(dev);
#endif

		throw std::runtime_error("Failed to initialize RX transciver");
	}
	SDR_RX(const SDR_RX &) = delete;
	SDR_RX &operator=(const SDR_RX &) = delete;
	virtual ~SDR_RX() {
#ifndef RTLSDR
		iio_buffer_destroy(rx_buf);
		iio_channel_disable(rx_ch_i);
		iio_channel_disable(rx_ch_q);
		iio_context_destroy(ctx);
#else
		delete [] temp;
		rtlsdr_close(dev);
#endif
	}
	std::size_t Poll() {
#ifndef RTLSDR
		ssize_t rx_bytes = iio_buffer_refill(rx_buf);
		if (rx_bytes < 0)
			return 0;

		/* ssize_t rx_samples = rx_bytes / iio_device_get_sample_size(rx_dev); */

		std::size_t i = 0;
		ptrdiff_t p_inc = iio_buffer_step(rx_buf);
		char *p_end = reinterpret_cast<char *>(iio_buffer_end(rx_buf));
		for (char *p_dat = reinterpret_cast<char *>(iio_buffer_first(rx_buf, rx_ch_i)); p_dat < p_end; p_dat += p_inc) {
			if (sink.GetFree() <= i)
				break;
			const int16_t ii = (reinterpret_cast<int16_t *>(p_dat))[0];
			const int16_t iq = (reinterpret_cast<int16_t *>(p_dat))[1];
			sink[sink.GetHeadIndex() + i].real(ii / static_cast<double>(SHRT_MAX));
			sink[sink.GetHeadIndex() + i].imag(iq / static_cast<double>(SHRT_MAX));
			i++;
		}
#else
		int bytes_read = 0;

		if (rtlsdr_read_sync(dev, temp, SAMPLES << 1, &bytes_read))
			return 0;

		std::size_t i;
		for (i = 0; i < static_cast<std::size_t>(bytes_read >> 1); i++) {
			if (sink.GetFree() <= i)
				break;
			sink[sink.GetHeadIndex() + i].real(static_cast<double>(temp[(i << 1) + 0]) / 127.5 - 1.0);
			sink[sink.GetHeadIndex() + i].imag(static_cast<double>(temp[(i << 1) + 1]) / 127.5 - 1.0);
		}
#endif

		sink.MoveHeadIndex(i);

		return i;
	}
private:
#ifndef RTLSDR
	iio_context *ctx;
	iio_device *rx_dev;
	iio_channel *rx_ch_i = nullptr, *rx_ch_q = nullptr;
	iio_buffer *rx_buf = nullptr;
#else
	rtlsdr_dev_t *dev;
	uint8_t *temp;
#endif
};

class Speaker {
public:
	Speaker() : buffer(nullptr), enabled(false) { }
	Speaker(const Speaker &) = delete;
	Speaker(Speaker &&) = delete;
	virtual ~Speaker() {
		Disable();
	}
	Speaker &operator=(const Speaker &) = delete;
	void Enable(const std::string &device, uint32_t sampling_rate, uint8_t channels, uint8_t channel_bits) {
		std::unique_lock<std::mutex> lock(access);

		if (enabled)
			return;

		enabled = true;
		lock.unlock();

		try {
			thread = std::thread(&Speaker::Thread, this, device, sampling_rate, channels, channel_bits);
		} catch (...) {
			lock.lock();
			enabled = false;
			throw;
		}
	}
	bool Disable() {
		std::unique_lock<std::mutex> lock(access);
		if (thread.joinable() && enabled) {
			enabled = false;
			lock.unlock();
			thread.join();
			return true;
		}
		return false;
	}
	bool IsEnabled() const {
		std::lock_guard<std::mutex> lock(access);
		return enabled;
	}
	std::string GetError() const {
		std::lock_guard<std::mutex> lock(access);
		return error;
	}
	std::size_t SetData(const uint8_t* data, std::size_t length) {
		std::lock_guard<std::mutex> lock(access);
		if (!buffer)
			return 0;
		length = std::min(length, buffer->GetFree());
		std::size_t bytes_left = length;
		while (bytes_left) {
			std::size_t step = (buffer->GetHeadIndex() + bytes_left >= buffer->GetLength()) ?
				buffer->GetLength() - buffer->GetHeadIndex() :
				bytes_left;
			std::memcpy(&buffer->GetHead(), data, step);
			buffer->MoveHeadIndex(step);
			bytes_left -= step;
		}
		return length;
	}
	std::size_t Consume(RingBuffer<double> &source, uint8_t channel_bits = 16) {
		std::lock_guard<std::mutex> lock(access);
		if (!buffer)
			return 0;
		std::size_t length = std::min(
			source.GetDataLength(),
			buffer->GetFree() / (channel_bits >> 3)
		);
		for (std::size_t i = 0; i < length; i++) {
			switch (channel_bits) {
			case 16:
				*reinterpret_cast<int16_t *>(&buffer->GetHead()) = (source.GetTail() < 1.) ?
					((source.GetTail() > -1.) ?
						static_cast<int16_t>(source.GetTail() * SHRT_MAX) :
						SHRT_MIN
					) :
					SHRT_MAX;
				break;
			case 8:
				buffer->GetHead() = (source.GetTail() < 1.) ? 
					((source.GetTail() > -1.) ?
						static_cast<uint8_t>((0.5 * (source.GetTail() + 1.)) * UCHAR_MAX) :
						0x00
					) :
					UCHAR_MAX;
				break;
			default:
				throw std::runtime_error("Unsupported channel bits value");
			}
			source.MoveTailIndex(1);
			buffer->MoveHeadIndex(channel_bits >> 3);
		}
		return length;
	}
private:
	void Thread(const std::string &device, uint32_t sampling_rate, uint8_t channels, uint8_t channel_bits) {
		snd_pcm_t *handle = nullptr;
		snd_pcm_hw_params_t *params = nullptr;

		try {
			if ((channel_bits != 8) && (channel_bits != 16))
				throw std::runtime_error("Unsupported channel bits value");

			int error = snd_pcm_open(&handle, device.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
			if (error < 0)
				throw std::runtime_error("Cannot open PCM device: " + device + " (" + std::string(snd_strerror(error)) + ")");

			snd_pcm_hw_params_alloca(&params);
			error = snd_pcm_hw_params_any(handle, params);
			if (error < 0)
				throw std::runtime_error("Cannot fill device configuration (" + std::string(snd_strerror(error)) + ")");

			error = snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
			if (error < 0)
				throw std::runtime_error("Cannot set device access type (" + std::string(snd_strerror(error)) + ")");

			error = snd_pcm_hw_params_set_format(handle, params, (channel_bits != 8) ? SND_PCM_FORMAT_S16_LE : SND_PCM_FORMAT_U8);
			if (error < 0)
				throw std::runtime_error("Cannot set channel bits value: " + std::to_string(channel_bits) + " (" + std::string(snd_strerror(error)) + ")");

			error = snd_pcm_hw_params_set_channels(handle, params, channels);
			if (error < 0)
				throw std::runtime_error("Cannot set channels number: " + std::to_string(channels) + " (" + std::string(snd_strerror(error)) + ")");

			unsigned rate = sampling_rate; int dir;
			error = snd_pcm_hw_params_set_rate_near(handle, params, &rate, &dir);
			if (error < 0)
				throw std::runtime_error("Cannot set sampling rate: " + std::to_string(sampling_rate) + " (" + std::string(snd_strerror(error)) + ")");

			/* if (rate != sampling_rate)
				throw std::runtime_error("Cannot set sampling rate: " + std::to_string(sampling_rate)); */

			snd_pcm_uframes_t frames = ALSA_PERIOD_DURATION_MS * sampling_rate / 1000;
			error = snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);
			if (error < 0)
				throw std::runtime_error("Cannot set period duration: " + std::to_string(ALSA_PERIOD_DURATION_MS) + "ms (" + std::string(snd_strerror(error)) + ")");

			frames *= ALSA_BUFFER_PERIODS;
			error = snd_pcm_hw_params_set_buffer_size_near(handle, params, &frames);
			if (error < 0)
				throw std::runtime_error("Cannot set buffer size: " + std::to_string(sampling_rate * ALSA_PERIOD_DURATION_MS * ALSA_BUFFER_PERIODS / 1000) + " (" + std::string(snd_strerror(error)) + ")");

			error = snd_pcm_hw_params(handle, params);
			if (error < 0)
				throw std::runtime_error("Cannot set hardware parameters (" + std::string(snd_strerror(error)) + ")");

			snd_pcm_hw_params_get_period_size(params, &frames, &dir);
			std::size_t size = frames * (channel_bits >> 3) * channels;

			std::unique_lock<std::mutex> lock(access);
			buffer = new RingBuffer<uint8_t>(4 * size);
			lock.unlock();

			std::size_t threshold = 2 * ALSA_PERIOD_DURATION_MS * sampling_rate / 1000 * (channel_bits >> 3) * channels;
			bool ready = false;
			while (IsEnabled()) {
				std::unique_lock<std::mutex> lock(access);
				if (!ready && (buffer->GetDataLength() >= threshold))
					ready = true;

				std::size_t bytes = (buffer->GetTailIndex() + buffer->GetDataLength() >= buffer->GetLength()) ?
					buffer->GetLength() - buffer->GetTailIndex() :
					buffer->GetDataLength(),
					count = bytes / (channel_bits >> 3) * channels;

				if (!count)
					goto wait;

				error = snd_pcm_avail(handle);
				if (error == -EPIPE) {
					/* EPIPE: Underrun */
					error = snd_pcm_prepare(handle);
					if (error < 0)
						throw std::runtime_error("Cannot exit from underrun (" + std::string(snd_strerror(error)) + ")");
					ready = false;
					continue;
				}

				if (error < 0)
					throw std::runtime_error("Cannot verify available frames (" + std::string(snd_strerror(error)) + ")");

				if (static_cast<snd_pcm_uframes_t>(error) < frames)
					goto wait;

				if (count > static_cast<std::size_t>(error)) {
					count = static_cast<std::size_t>(error);
					bytes = count * (channel_bits >> 3) * channels;
				}

				error = snd_pcm_writei(handle, &buffer->GetTail(), count);
				if (error == -EPIPE) {
					/* EPIPE: Underrun */
					error = snd_pcm_prepare(handle);
					if (error < 0)
						throw std::runtime_error("Cannot exit from underrun (" + std::string(snd_strerror(error)) + ")");
					ready = false;
					continue;
				}

				if (error < 0)
					throw std::runtime_error("Error while writing to device (" + std::string(snd_strerror(error)) + ")");

				buffer->MoveTailIndex(bytes);

				continue;

wait:
				lock.unlock();

				std::this_thread::sleep_for(std::chrono::milliseconds(std::max(static_cast<int>(0.5 * 1000 * frames / sampling_rate), 1)));
			}
		} catch (std::exception& catched) {
			std::lock_guard<std::mutex> lock(access);
			error = catched.what();
		}

		/* if (handle) {
			snd_pcm_drain(handle);
			snd_pcm_close(handle);
		} */
		std::lock_guard<std::mutex> lock(access);
		if (buffer) {
			delete buffer;
			buffer = nullptr;
		}
	}
	mutable std::mutex access;
	RingBuffer<uint8_t> *buffer;
	std::thread thread;
	std::string error;
	bool enabled;
};

template <typename T>
class FIR_Filter : public Entity<T> {
public:
	FIR_Filter() = delete;
	FIR_Filter(const double coefficients[], std::size_t coeffs_size, std::size_t decimate = 1, std::size_t sink_length = SAMPLES) : Entity<T>(sink_length), coefficients(coefficients), coeffs_size(coeffs_size), decimate(decimate) { }
	std::size_t Process(RingBuffer<T> &source) {
		std::size_t count = 0;
		while (source.GetDataLength() >= std::max(coeffs_size, decimate) && Entity<T>::sink.GetFree()) {
			T sum = 0;
			for (std::size_t k = 0; k < coeffs_size; k++) {
				std::size_t src_idx = source.GetTailIndex() + coeffs_size - (k + 1);
				sum += source[src_idx] * coefficients[k];
			}
			source.MoveTailIndex(decimate);

			Entity<T>::sink.GetHead() = sum;
			Entity<T>::sink.MoveHeadIndex(1);
			count++;
		}
		return count;
	}
private:
	const double *coefficients;
	std::size_t coeffs_size, decimate;
};

class FM_Demodulator : public Entity<double> {
public:
	FM_Demodulator() = delete;
	FM_Demodulator(uint32_t sampling_rate, uint32_t bandwidth = MHZ(0.2), std::size_t sink_length = SAMPLES) : Entity(sink_length), sampling_rate(sampling_rate), bandwidth(bandwidth) { }
	std::size_t Process(RingBuffer<std::complex<double>> &source) {
		std::size_t count = 0;
		while ((source.GetDataLength() >= 2) && sink.GetFree()) {
			std::complex<double> *s1 = &source[source.GetTailIndex()],
				*s2 = &source[source.GetTailIndex() + 1];
			source.MoveTailIndex(1);

			double diff = std::arg((*s2) * std::conj(*s1));
			sink.GetHead() = diff * sampling_rate / (2. * M_PI * static_cast<double>(bandwidth >> 1));
			sink.MoveHeadIndex(1);
			count++;
		}
		return count;
	}
private:
	uint32_t sampling_rate, bandwidth;
};

class Deemphasis : public Entity<double> {
public:
	Deemphasis() = delete;
	Deemphasis(uint32_t sampling_rate, double tau, std::size_t sink_length = SAMPLES) : Entity(sink_length), prev(0.) {
		double T = 1. / static_cast<double>(sampling_rate);
		alpha = T / (tau + T);
	}
	std::size_t Process(RingBuffer<double> &source) {
		std::size_t count = 0;
		while (source.GetDataLength() && sink.GetFree()) {
			prev = alpha * source.GetTail() + (1. - alpha) * prev;
			source.MoveTailIndex(1);

			sink.GetHead() = prev;
			sink.MoveHeadIndex(1);
			count++;
		}
		return count;
	}
private:
	double alpha, prev;
};

static bool stop = false;

void SignalHandler(int sigNum) {
    stop = true;
}

int main(int argc, char** argv) {
#ifndef RTLSDR
	std::string uri = "ip:192.168.2.1",
#else
	std::string uri = "0",
#endif
		speaker_dev = "default";
	long long freq = LO_FREQ;
	int opt;

	while ((opt = getopt(argc, argv, "f:d:s:")) != -1) {
		switch (opt) {
		case 'f':
			freq = MHZ(std::stod(optarg));
			break;
		case 'd':
			uri = std::string(optarg);
			break;
		case 's':
			speaker_dev = std::string(optarg);
			break;
		}
	}

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

	try {
		if (SAMPLE_FREQ % (AUDIO_SAMPLE_RATE * SAMPLE_DEC))
			throw std::runtime_error("Invalid audio sampling rate");

		SDR_RX radio(uri, freq);

		FIR_Filter<std::complex<double>> lowpass(lowpass_filter_coeffs, ARRAY_LENGTH(lowpass_filter_coeffs), SAMPLE_DEC);

		FM_Demodulator demod(SAMPLE_FREQ / SAMPLE_DEC, BANDWIDTH);

		FIR_Filter<double> audio_lowpass(audio_filter_coeffs, ARRAY_LENGTH(audio_filter_coeffs), SAMPLE_FREQ / (AUDIO_SAMPLE_RATE * SAMPLE_DEC));

		/* tau=50us */
		Deemphasis deemp(AUDIO_SAMPLE_RATE, 50. / 1000000.);

		Speaker speaker;
		speaker.Enable(speaker_dev, AUDIO_SAMPLE_RATE, 1, AUDIO_SAMPLE_BITS);

		while (!stop) {
			radio.Poll();
			lowpass.Process(radio.GetSinkBuffer());
			demod.Process(lowpass.GetSinkBuffer());
			audio_lowpass.Process(demod.GetSinkBuffer());
			deemp.Process(audio_lowpass.GetSinkBuffer());
			if (!speaker.GetError().empty())
				throw std::runtime_error(speaker.GetError());
			speaker.Consume(deemp.GetSinkBuffer(), AUDIO_SAMPLE_BITS);
		}
	} catch (std::exception &error) {
		std::cerr << error.what() << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
