"""Build sdcard/VOICE.WAV from a raw spoken announcement.

    python scripts/make_voice_asset.py raw.wav sdcard/VOICE.WAV stream 60

raw.wav comes from scripts/make_voice_wav.ps1 (Windows System.Speech, 16 bit
PCM, mono, 8 kHz).  Leading and trailing silence is trimmed, the airport
boarding chime is prepended, and the result is written in the format the FPGA
loader expects.

The ADV04 streaming path has no 40960 sample bank limit.  The optional third
argument "stream" disables the old bank check, and the optional fourth argument
sets a minimum duration in seconds.
"""

import sys
import wave

import numpy as np

BANK_WORDS = 40960
RATE = 8000

THRESHOLD = 200          # silence gate, in 16 bit counts
LEAD_MS = 30             # kept before the first loud sample
TAIL_MS = 60             # kept after the last loud sample

# The monitor speakers are quiet and the WAV chime is much louder than the
# speech.  A peak-only normalise left the announcement around -30 dBFS, which
# is easily mistaken for "no voice" after the -17 dBFS chime.  Use a gentle
# compressor with a fixed make-up gain instead: quiet speech is raised, while
# the few high peaks are limited rather than wrapped.
COMP_THRESH_DB = -18.0
COMP_RATIO = 8.0
COMP_MAKEUP_DB = 15.0
COMP_CEILING = 32111.0

CHIME_LEVEL = 0.70       # chime peak, fraction of full scale
CHIME_NOTES = [(659.26, 0.25), (523.25, 0.30)]   # E5, then C5 as in the RTL
CHIME_GAP_MS = 120


def bell(freq, seconds):
    n = int(round(seconds * RATE))
    t = np.arange(n, dtype=np.float64) / RATE
    # one raised cosine cycle as the bell envelope
    env = 0.5 - 0.5 * np.cos(2.0 * np.pi * np.arange(n) / n)
    return CHIME_LEVEL * 32767.0 * env * np.sin(2.0 * np.pi * freq * t)


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
stream_mode = (len(sys.argv) > 3) and (sys.argv[3].lower() == "stream")
min_seconds = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0

with wave.open(src, "rb") as w:
    assert w.getnchannels() == 1, "expected mono"
    assert w.getsampwidth() == 2, "expected 16 bit PCM"
    assert w.getframerate() == RATE, f"expected {RATE} Hz"
    voice = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.int32)

voice = trim(voice)

# Raise the speech with a compressor.  The chime is prepended afterwards and
# keeps its own level, so this cannot make the ding-dong quieter or louder.
voice = compress(voice.astype(np.float64))

chime = np.concatenate([bell(f, s) for f, s in CHIME_NOTES])
chime = np.concatenate([chime, np.zeros(CHIME_GAP_MS * RATE // 1000)])

samples = np.concatenate([chime, voice])
duration = samples.size / RATE
if (not stream_mode) and (samples.size > BANK_WORDS):
    raise SystemExit(
        f"{samples.size} samples does not fit the {BANK_WORDS} sample bank "
        f"({BANK_WORDS / RATE:.2f} s at {RATE} Hz); shorten the text or raise "
        f"the speaking rate"
    )
if duration < min_seconds:
    raise SystemExit(
        f"{duration:.2f} s is shorter than the requested {min_seconds:.2f} s; "
        f"lengthen the text or lower the speaking rate"
    )

with wave.open(dst, "wb") as w:
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(RATE)
    w.writeframes(np.clip(samples, -32768, 32767).astype("<i2").tobytes())

print(
    f"{dst}: chime {chime.size / RATE:.2f}s + voice {voice.size / RATE:.2f}s "
    f"= {samples.size} samples ({duration:.2f}s)"
    + ("" if stream_mode else f" of {BANK_WORDS}")
)
