// Raw RX audio input with digital downconversion for ardopcf.
//
// An external process taps a shared wideband receiver and pipes a stream of
// headerless signed 16-bit little-endian mono samples to ardopcf (via stdin, a
// FIFO, a socket pathname, or a file).  The ARDOP signal may sit anywhere in
// that passband, so this module:
//
//   1. complex-mixes the input by an NCO at (offset - 1500) Hz, translating the
//      configured center frequency down to ARDOP's nominal 1500 Hz center,
//   2. applies a complex band-pass FIR (a low-pass prototype modulated to 1500
//      Hz) that isolates the ARDOP band and rejects the mixing image,
//   3. decimates by inrate / 12000 to the modem's 12 kHz sample rate, and
//   4. takes the real part and feeds 12 kHz samples to ProcessNewSamples().
//
// The modem's existing leader search still fine-tunes residual offset within
// TuningRange, so --rxoffset only has to be approximately right.
//
// Raw input is receive-only; transmit (when enabled, e.g. for KISS) continues to
// use the normal playback device and PTT.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef WIN32
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#endif

#include "os_util.h"
#include "ARDOPC.h"
#include "ardopcommon.h"
#include "RawInput.h"

#define MODEM_RATE 12000
#define ARDOP_CENTER 1500.0
#define MAX_TAPS 512

extern bool RXEnabled;
extern bool Capturing;
void ProcessNewSamples(short *Samples, int nSamples);
bool PreprocessNewSamples(short *Samples, int nSamples);

// Configuration, set by RawInputConfig().
static char RawPath[256] = "";  // empty means raw input disabled
static int RawInRate = 48000;
static double RawOffsetHz = ARDOP_CENTER;

// Runtime state.
static int RawFd = -1;
static bool RawEof = false;

// NCO (complex local oscillator at offset - 1500 Hz).
static double NcoPhase = 0.0;
static double NcoPhaseInc = 0.0;

// Decimation.
static int Decim = 4;
static int DecimPhase = 0;

// Complex band-pass FIR coefficients and complex sample history (ring buffer).
static int NTaps = 0;
static float TapR[MAX_TAPS];
static float TapI[MAX_TAPS];
static float HistR[MAX_TAPS];
static float HistI[MAX_TAPS];
static int HistPos = 0;

// Output accumulation: full ReceiveSize blocks are handed to the modem.
static short OutBuf[ReceiveSize];
static int OutCount = 0;

// One leftover input byte when a read returns an odd number of bytes.
static unsigned char PendingByte = 0;
static bool HavePendingByte = false;

bool RawInputConfig(const char *path, int inrate, double offsetHz)
{
	if (path == NULL || path[0] == 0x00)
		return false;

	if (strlen(path) >= sizeof(RawPath)) {
		ZF_LOGE("RawInput: source path too long.");
		return false;
	}

	if (inrate <= 0 || inrate % MODEM_RATE != 0) {
		ZF_LOGE("RawInput: input rate %d Hz must be a positive multiple of %d.",
			inrate, MODEM_RATE);
		return false;
	}

	strcpy(RawPath, path);  // length checked above
	RawInRate = inrate;
	RawOffsetHz = offsetHz;
	Decim = inrate / MODEM_RATE;
	return true;
}

bool RawInputActive()
{
	return RawPath[0] != 0x00;
}

// Build a complex band-pass FIR: a windowed-sinc low-pass prototype (cutoff
// chosen to pass the ~2 kHz ARDOP band) modulated up to ARDOP_CENTER so it
// passes positive frequencies around 1500 Hz and rejects the mixing image.
static void BuildTaps()
{
	// Number of taps scales with the decimation factor; odd for a symmetric,
	// linear-phase prototype.
	NTaps = 16 * Decim + 1;
	if (NTaps > MAX_TAPS)
		NTaps = MAX_TAPS | 1;  // keep odd

	double fc = 1350.0;  // low-pass prototype cutoff (Hz): half the ~2.7 kHz BW
	double M = (NTaps - 1) / 2.0;

	for (int k = 0; k < NTaps; k++) {
		double n = k - M;
		// Windowed-sinc low-pass prototype.
		double lp;
		if (n == 0.0)
			lp = 2.0 * fc / RawInRate;
		else
			lp = sin(2.0 * M_PI * fc / RawInRate * n) / (M_PI * n);
		// Hamming window.
		lp *= 0.54 - 0.46 * cos(2.0 * M_PI * k / (NTaps - 1));
		// Modulate the real low-pass up to ARDOP_CENTER to form a complex
		// band-pass centered at +1500 Hz.
		double w = 2.0 * M_PI * ARDOP_CENTER / RawInRate * n;
		TapR[k] = (float)(lp * cos(w));
		TapI[k] = (float)(lp * sin(w));
	}
}

bool RawInputOpen()
{
	if (!RawInputActive())
		return false;

#ifdef WIN32
	ZF_LOGE("RawInput: raw audio input is not supported on Windows.");
	return false;
#else
	if (strcmp(RawPath, "-") == 0) {
		RawFd = 0;  // stdin
	} else {
		RawFd = open(RawPath, O_RDONLY | O_NONBLOCK);
		if (RawFd < 0) {
			ZF_LOGE("RawInput: cannot open \"%s\": %s", RawPath, strerror(errno));
			return false;
		}
	}
	// Ensure non-blocking reads so the main loop is never stalled.
	int fl = fcntl(RawFd, F_GETFL, 0);
	if (fl != -1)
		fcntl(RawFd, F_SETFL, fl | O_NONBLOCK);

	NcoPhase = 0.0;
	NcoPhaseInc = 2.0 * M_PI * (RawOffsetHz - ARDOP_CENTER) / RawInRate;
	DecimPhase = 0;
	HistPos = 0;
	OutCount = 0;
	HavePendingByte = false;
	RawEof = false;
	memset(HistR, 0, sizeof(HistR));
	memset(HistI, 0, sizeof(HistI));
	BuildTaps();

	RXEnabled = true;  // the main loop must poll us even with no sound card open

	ZF_LOGI("RawInput: reading s16le from %s at %d Hz, translating %.1f Hz ->"
		" %.0f Hz, decimating by %d to %d Hz (%d FIR taps).",
		strcmp(RawPath, "-") == 0 ? "stdin" : RawPath, RawInRate, RawOffsetHz,
		ARDOP_CENTER, Decim, MODEM_RATE, NTaps);
	return true;
#endif
}

// Push one 12 kHz output sample, flushing full ReceiveSize blocks to the modem.
static void EmitSample(float value)
{
	long v = lrintf(value);
	if (v > 32767)
		v = 32767;
	else if (v < -32768)
		v = -32768;
	OutBuf[OutCount++] = (short)v;

	if (OutCount >= ReceiveSize) {
		if (Capturing)
			ProcessNewSamples(OutBuf, ReceiveSize);
		else
			// Mirror the sound backends: keep the RX WAV (when enabled) time
			// continuous while not capturing (e.g. during transmit).
			PreprocessNewSamples(OutBuf, ReceiveSize);
		OutCount = 0;
	}
}

// Run one input sample through the NCO, FIR and decimator.
static void ProcessInputSample(short sample)
{
	double x = (double)sample;

	// Complex downconvert: multiply by exp(-j * NcoPhase).
	float mixR = (float)(x * cos(NcoPhase));
	float mixI = (float)(-x * sin(NcoPhase));
	NcoPhase += NcoPhaseInc;
	if (NcoPhase > M_PI)
		NcoPhase -= 2.0 * M_PI;
	else if (NcoPhase < -M_PI)
		NcoPhase += 2.0 * M_PI;

	// Store newest sample in the ring buffer.
	HistPos = (HistPos + 1) % NTaps;
	HistR[HistPos] = mixR;
	HistI[HistPos] = mixI;

	// Only compute an output sample every Decim input samples.
	if (++DecimPhase < Decim)
		return;
	DecimPhase = 0;

	// Real part of the complex FIR output: Re{ sum h[k] * z[n-k] }.
	float acc = 0.0f;
	int idx = HistPos;
	for (int k = 0; k < NTaps; k++) {
		acc += TapR[k] * HistR[idx] - TapI[k] * HistI[idx];
		idx--;
		if (idx < 0)
			idx = NTaps - 1;
	}

	// The single-sideband mix halves amplitude; restore unit gain with x2.
	EmitSample(2.0f * acc);
}

void RawInputPoll()
{
#ifndef WIN32
	if (!RawInputActive() || RawFd < 0 || RawEof)
		return;

	unsigned char buf[8192];
	// Bound the work per poll so the main loop stays responsive even if a large
	// backlog is available; real-time 48 kHz is ~96 kB/s, far below this.
	int budget = 16;  // up to 16 * 8192 = 128 kB per poll

	while (budget-- > 0) {
		ssize_t n = read(RawFd, buf, sizeof(buf));
		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK)
				break;  // no data available right now
			if (errno == EINTR)
				continue;
			ZF_LOGE("RawInput: read error on source, stopping: %s",
				strerror(errno));
			RawEof = true;
			return;
		}
		if (n == 0) {
			ZF_LOGW("RawInput: end of sample stream.");
			RawEof = true;
			return;
		}

		int i = 0;
		// Complete a short split across the previous read.
		if (HavePendingByte) {
			short s = (short)((unsigned short)PendingByte
				| ((unsigned short)buf[0] << 8));
			ProcessInputSample(s);
			HavePendingByte = false;
			i = 1;
		}
		for (; i + 1 < n; i += 2) {
			short s = (short)((unsigned short)buf[i]
				| ((unsigned short)buf[i + 1] << 8));
			ProcessInputSample(s);
		}
		if (i < n) {
			PendingByte = buf[i];
			HavePendingByte = true;
		}

		if (n < (ssize_t)sizeof(buf))
			break;  // drained what was available
	}
#endif
}
