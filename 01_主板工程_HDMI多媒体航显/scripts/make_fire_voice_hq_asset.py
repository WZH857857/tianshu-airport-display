"""Clean and level a 16 kHz fire-evacuation announcement.

The FPGA reads the sample rate from the WAV header, and its streaming voice
player already supports 16 kHz.  Staying at 16 kHz avoids the aliasing from
the old 8 kHz System.Speech output while keeping the file in a format that
the current bitstream plays without rebuilding.

    python scripts/make_fire_voice_hq_asset.py raw-16k.wav sdcard/FIRE.WAV
"""

import sys
import wave

import numpy as np
from scipy.signal import butter, resample_poly, sosfilt

INPUT_RATE = 16000
OUTPUT_RATE = 16000
THRESHOLD = 180
LEAD_MS = 30
TAIL_MS = 100

HIGHPASS_HZ = 90.0
LOWPASS_HZ = 6800.0

COMP_THRESH_DB = -20.0
COMP_RATIO = 3.0
COMP_MAKEUP_DB = 8.0
TARGET_RMS = 7600.0
PEAK_CEILING = 32111.0
LIMIT_KNEE = 0.82 * 32767.0


def trim(pcm: np.ndarray) -> np.ndarray:
    loud = np.flatnonzero(np.abs(pcm) > THRESHOLD)
    if not loud.size:
        return pcm
    lo = max(0, int(loud[0]) - LEAD_MS * INPUT_RATE // 1000)
    hi = min(pcm.size, int(loud[-1]) + TAIL_MS * INPUT_RATE // 1000)
    return pcm[lo:hi]


def compressor(pcm: np.ndarray) -> np.ndarray:
    threshold = 10.0 ** (COMP_THRESH_DB / 20.0) * 32768.0
    makeup = 10.0 ** (COMP_MAKEUP_DB / 20.0)
    magnitude = np.abs(pcm)
    compressed = np.where(
        magnitude <= threshold,
        magnitude * makeup,
        (threshold + (magnitude - threshold) / COMP_RATIO) * makeup,
    )
    return np.sign(pcm) * compressed


def soft_limit(pcm: np.ndarray) -> np.ndarray:
    """Smoothly reduce only the highest peaks, so speech stays intelligible."""
    magnitude = np.abs(pcm)
    over = magnitude > LIMIT_KNEE
    if not np.any(over):
        return pcm
    limited = pcm.copy()
    scale = (PEAK_CEILING - LIMIT_KNEE)
    limited[over] = np.sign(pcm[over]) * (
        LIMIT_KNEE + scale * np.tanh((magnitude[over] - LIMIT_KNEE) / scale)
    )
    return limited


src, dst = sys.argv[1], sys.argv[2]
if len(sys.argv) > 3:
    OUTPUT_RATE = int(sys.argv[3])

with wave.open(src, "rb") as w:
    assert w.getnchannels() == 1, "expected mono"
    assert w.getsampwidth() == 2, "expected 16 bit PCM"
    assert w.getframerate() == INPUT_RATE, f"expected {INPUT_RATE} Hz"
    pcm = np.frombuffer(
        w.readframes(w.getnframes()), dtype="<i2"
    ).astype(np.float64)

pcm = trim(pcm)

# Remove DC/rumble and the harsh upper band before the dynamic-range stage.
hp = butter(2, HIGHPASS_HZ, btype="highpass", fs=INPUT_RATE, output="sos")
cutoff = min(LOWPASS_HZ, OUTPUT_RATE * 0.42)
lp = butter(4, cutoff, btype="lowpass", fs=INPUT_RATE, output="sos")
pcm = sosfilt(hp, pcm)
pcm = sosfilt(lp, pcm)

pcm = compressor(pcm)
pcm = soft_limit(pcm)

# The board's proven announcement path is 8 kHz.  Resample only after the
# anti-alias filter above, then normalise at the final rate.
if OUTPUT_RATE != INPUT_RATE:
    pcm = resample_poly(pcm, OUTPUT_RATE, INPUT_RATE)

# Match the normal announcement's level while leaving headroom for the
# mu-law round trip and the board's amplifier.
current_rms = float(np.sqrt(np.mean(pcm * pcm)))
if current_rms > 0.0:
    pcm *= TARGET_RMS / current_rms

peak = float(np.max(np.abs(pcm)))
pcm = soft_limit(pcm)

samples = np.clip(np.rint(pcm), -32768, 32767).astype("<i2")

with wave.open(dst, "wb") as w:
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(OUTPUT_RATE)
    w.writeframes(samples.tobytes())

final_rms = float(np.sqrt(np.mean(samples.astype(np.float64) ** 2)))
print(
    f"{dst}: {samples.size} samples, {samples.size / OUTPUT_RATE:.2f} s, "
    f"{OUTPUT_RATE} Hz, "
    f"RMS {final_rms:.0f}, peak {np.max(np.abs(samples))}"
)
