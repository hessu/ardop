#ifndef RAWINPUT_H
#define RAWINPUT_H

#include <stdbool.h>

// Raw RX audio input with digital downconversion (see RawInput.c).
//
// Reads headerless signed 16-bit little-endian mono audio samples from a file,
// FIFO, socket pathname, or stdin, digitally downconverts a configurable audio
// center frequency to ARDOP's nominal 1500 Hz center, low-pass filters and
// decimates to the modem's 12 kHz sample rate, and feeds the result to
// ProcessNewSamples().  This lets ardopcf act as one of several demodulators
// sharing a single wideband receiver.

// Configure raw input.  path is the sample source: "-" for stdin, otherwise a
// file, FIFO, or socket pathname.  inrate is the input sample rate in Hz (a
// positive integer multiple of 12000).  offsetHz is the audio frequency in the
// input stream that is translated to ARDOP's nominal 1500 Hz center.  Returns
// false (leaving raw input disabled) on invalid arguments.
bool RawInputConfig(const char *path, int inrate, double offsetHz);

// True if raw input was configured, so the main loop polls it instead of the
// sound-card capture device.
bool RawInputActive();

// Open the configured source and initialise the downconverter.  Call once before
// the main loop.  Returns false on failure.
bool RawInputOpen();

// Read whatever samples are available, downconvert and decimate them to 12 kHz,
// and pass them to ProcessNewSamples() (or PreprocessNewSamples() while not
// capturing).  Non-blocking; safe to call every main-loop iteration.
void RawInputPoll();

#endif  // RAWINPUT_H
