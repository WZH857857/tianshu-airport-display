"""Model the FPGA voice chain and write what the HDMI audio island will send.

    python scripts/make_fpga_voice_preview.py sdcard/VOICE.WAV build/

The model is a bit-exact transcription of the RTL:

    * ulaw_encode / ulaw_decode from rtl/audio/ulaw_codec.v
    * the 16.16 phase accumulator, the two entry queue and the linear
      interpolation of rtl/audio/voice_player_stream.v (GAIN_SHIFT = 7)

It exists so the sound the board should make can be listened to without a
board: if the board sounds different from these files, the difference is in
the card, the level or the analogue path, not in the digital chain.
"""

import sys
import wave

import numpy as np

RATE_SRC = 8000
RATE_FRAME = 48000
PHASE_INC = 10923
GAIN_SHIFT = 7
BIAS = 132
CLIP = 32635


def ulaw_encode(pcm):
    """rtl/audio/ulaw_codec.v ulaw_encode, vectorised."""
    pcm = pcm.astype(np.int64)
    sign = (pcm < 0).astype(np.int64)
    mag = np.minimum(np.abs(pcm), CLIP)
    biased = mag + BIAS
    exp = np.clip(np.floor(np.log2(biased)).astype(np.int64) - 7, 0, 7)
    mant = (biased >> (exp + 3)) & 0xF
    code = ~((sign << 7) | (exp << 4) | mant) & 0xFF
    return code.astype(np.uint8)


def ulaw_decode(code):
    """rtl/audio/ulaw_codec.v ulaw_decode, vectorised."""
    u = (~code.astype(np.int64)) & 0xFF
    sign = (u >> 7) & 1
    exp = (u >> 4) & 7
    mant = u & 0xF
    mag = ((((mant << 3) + BIAS) << exp) - BIAS) & 0x7FFF
    pcm = np.where(sign == 1, -mag, mag)
    return pcm.astype(np.int64)


def player(samples):
    """rtl/audio/voice_player_stream.v, frame by frame.

    Frame f presents the source position f * PHASE_INC / 65536 to the
    interpolator.  The RTL normalises a wrapped phase at the start of the
    frame it wraps in and then mixes with the eight bit fraction of the
    normalised phase, so the output is a read of that mathematical position.
    Deriving the position directly avoids the old model's fault, where the
    wrap frame jumped a whole source sample ahead and every sixth frame was
    duplicated, which is what roughened speech into the mangled sound heard
    on the board.
    """
    src = np.asarray(samples, dtype=np.int64)
    n = src.size
    if n == 0:
        return np.zeros(0, dtype=np.int64)

    frames = ((n - 1) * 0x10000) // PHASE_INC + 2
    pos = np.arange(frames, dtype=np.int64) * PHASE_INC
    idx = np.minimum(pos >> 16, n - 1)
    frac = (pos & 0xFFFF) >> 8

    # The RTL holds the final sample once its queue drains.
    nxt = np.minimum(idx + 1, n - 1)
    mixed = src[idx] + (((src[nxt] - src[idx]) * frac) >> 8)
    return mixed << GAIN_SHIFT


def write_wav(path, data16, rate):
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(np.clip(data16, -32768, 32767).astype("<i2").tobytes())


src_path, out_dir = sys.argv[1], sys.argv[2]

with wave.open(src_path, "rb") as w:
    assert w.getnchannels() == 1 and w.getsampwidth() == 2
    assert w.getframerate() == RATE_SRC
    pcm = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.int64)

code = ulaw_encode(pcm)
back = ulaw_decode(code)

# Codec loss: how far the mu-law round trip is from the original speech.
err = back - pcm
snr = 10.0 * np.log10(np.sum(pcm.astype(np.float64) ** 2) /
                      max(np.sum(err.astype(np.float64) ** 2), 1.0))
print(f"source     : {pcm.size} samples ({pcm.size / RATE_SRC:.2f} s), "
      f"peak {int(np.max(np.abs(pcm)))}, rms {np.sqrt(np.mean(pcm.astype(float) ** 2)):.0f}")
print(f"mu-law     : {code.size} bytes, round trip SNR {snr:.1f} dB, "
      f"max error {int(np.max(np.abs(err)))}")

frames = player(back)
audio16 = frames >> 8
print(f"player     : {frames.size} frames ({frames.size / RATE_FRAME:.2f} s), "
      f"peak {int(np.max(np.abs(frames)))} (24 bit)")

# Two listenable excerpts: the chime and the opening, then a stretch of the
# middle of the announcement.
for name, start in (("fpga_out_start.wav", 0.0),
                    ("fpga_out_middle.wav", 45.0)):
    lo = int(start * RATE_FRAME)
    hi = min(lo + 30 * RATE_FRAME, audio16.size)
    write_wav(f"{out_dir}/{name}", audio16[lo:hi], RATE_FRAME)
    print(f"wrote {out_dir}/{name}: {start:.0f}s .. {hi / RATE_FRAME:.0f}s")

write_wav(f"{out_dir}/fpga_out_full.wav", audio16, RATE_FRAME)
print(f"wrote {out_dir}/fpga_out_full.wav")
