"""Build sdcard/FIRE.WAV from the raw spoken fire announcement.

    python scripts/make_fire_voice_asset.py build/fire_voice_raw.wav sdcard/FIRE.WAV 10

Same audio pipeline as the normal announcement (trim, gentle compressor, 16
bit PCM, mono, 8 kHz), but without the boarding chime: the alarm buzzer is
already sounding, so the message must start immediately.  The optional third
argument sets a minimum duration in seconds.
"""

import sys
import wave

import numpy as np

RATE = 8000

THRESHOLD = 200          # silence gate, in 16 bit counts
LEAD_MS = 20             # kept before the first loud sample
TAIL_MS = 80             # kept after the last loud sample

COMP_THRESH_DB = -18.0
COMP_RATIO = 8.0
COMP_MAKEUP_DB = 15.0
COMP_CEILING = 32111.0


def trim(pcm):
    loud = np.flatnonzero(np.abs(pcm) > THRESHOLD)
    if not loud.size:
        return pcm
    lo = max(0, int(loud[0]) - LEAD_MS * RATE // 1000)
    hi = min(pcm.size, int(loud[-1]) + TAIL_MS * RATE // 1000)
    return pcm[lo:hi]


def compress(pcm):
    threshold = 10.0 ** (COMP_THRESH_DB / 20.0) * 32768.0
    gain = 10.0 ** (COMP_MAKEUP_DB / 20.0)
    magnitude = np.abs(pcm)
    compressed = np.sign(pcm) * np.where(
        magnitude <= threshold,
        magnitude * gain,
        threshold * gain +
        (magnitude - threshold) * gain / COMP_RATIO,
    )
    return np.clip(compressed, -COMP_CEILING, COMP_CEILING)


src, dst = sys.argv[1], sys.argv[2]
min_seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

with wave.open(src, "rb") as w:
    assert w.getnchannels() == 1, "expected mono"
    assert w.getsampwidth() == 2, "expected 16 bit PCM"
    assert w.getframerate() == RATE, f"expected {RATE} Hz"
    voice = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.int32)

voice = trim(voice)
voice = compress(voice.astype(np.float64))

samples = np.clip(voice, -32768, 32767).astype("<i2")
duration = samples.size / RATE
if duration < min_seconds:
    raise SystemExit(
        f"{duration:.2f} s is shorter than the requested {min_seconds:.2f} s; "
        f"lengthen the text or lower the speaking rate"
    )

with wave.open(dst, "wb") as w:
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(RATE)
    w.writeframes(samples.tobytes())

print(f"{dst}: {samples.size} samples ({duration:.2f} s)")
