# Simple FM Audio Demodulator
This simple FM audio demodulator is designed for use with SDR platforms such as **ADALM-Pluto** or **RTL-SDR**. It is a console-only application.

## Dependencies

Install the appropriate development libraries depending on your SDR platform:
* **ADALM-Pluto**: `libiio-dev`
* **RTL-SDR**: `librtlsdr-dev`

## Installation

To compile and run the program, first install the required dependencies.
Example for RTL-SDR on **Ubuntu 24.04**:
```
sudo apt update
sudo apt install build-essential librtlsdr-dev
```

Then change to the project directory, e.g.:
```
cd ~/sdr_fm_rx
```

Compile using `make` (add `RTLSDR=1` for RTL-SDR builds):
```
make RTLSDR=1
```

## Running the Demodulator

Example: receive 101.2 MHz FM broadcast:
```
./fm_rx -f 101200000
```

You should hear the demodulated audio through your PC speakers.

## Parameters

* `-f` — Center frequency in Hz, e.g. `101200000` for 101.2 MHz
* `-d` — SDR device
  * Default for RTL-SDR: `0`, uses device index
  * Default for Pluto: `ip:192.168.2.1`
* `-s` — ALSA PCM output device
  * Default: `default`
  * Use `hw:0,0` or `plughw:0,0` if no sound is heard
