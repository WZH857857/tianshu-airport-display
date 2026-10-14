"""Golden model for the ADV04 voice path.

Reads VOICE.WAV, compresses it with the same G.711 encoder the FPGA uses, then
replays it through a bit accurate model of voice_player, so the numbers the
end to end bench has to hit are available without a simulator.

The model also keeps the old 18 bit gain path: that is the version the first
on-board build shipped with, and it is what turns a full scale mu-law code into
a wrapped value instead of 4111872.

  python sim\voice_golden.py
"""
import os
import struct
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
WAV = os.path.join(PROJ, "VOICE.WAV")
HEX = os.path.join(HERE, "build_e2e", "voice_bank.hex")

BIAS = 132
CLIP = 32635
PHASE_INC = 10923          # 8 kHz source at 48 kHz frames, 16.16
AW = 16
FRAMES = 30000             # 625 ms, what tb_voice_e2e plays


def encode(pcm):
    """G.711 compressor, tracking ulaw_encode in rtl/audio/ulaw_codec.v."""
    sign = 1 if pcm < 0 else 0
    mag = -pcm if pcm < 0 else pcm
    if mag > CLIP:
        mag = CLIP
    biased = mag + BIAS
    if biased & 0x8000:
        exp = 7
    elif biased & 0x4000:
        exp = 7
    elif biased & 0x2000:
        exp = 6
    elif biased & 0x1000:
        exp = 5
    elif biased & 0x0800:
        exp = 4
    elif biased & 0x0400:
        exp = 3
    elif biased & 0x0200:
        exp = 2
    elif biased & 0x0100:
        exp = 1
    else:
        exp = 0
    mant = (biased >> (exp + 3)) & 0xF
    return (~((sign << 7) | (exp << 4) | mant)) & 0xFF


def decode(code):
    """G.711 expander, tracking ulaw_decode in rtl/audio/ulaw_codec.v."""
    u = (~code) & 0xFF
    sign = (u >> 7) & 1
    exp = (u >> 4) & 7
    mant = u & 0xF
    mag = (((mant << 3) + 0x84) << exp) - 0x84
    mag &= 0x7FFF
    return -mag if sign else mag


def wrap(value, bits):
    mask = (1 << bits) - 1
    value &= mask
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value


def play(bank, frames, words, gain_shift, gain_bits=24):
    """One 48 kHz frame per iteration, the way voice_player walks the bank."""
    out = []
    phase = 0
    for _ in range(frames):
        idx = (phase >> 16) & ((1 << AW) - 1)
        idx_wrap = idx if (idx + 1) >= words else idx + 1
        s0 = decode(bank[idx])
        s1 = decode(bank[idx_wrap])
        frac = (phase >> 8) & 0xFF
        delta = s1 - s0
        prod = delta * (frac >> 4)
        mixed = wrap(s0 + (prod >> 4), 17)
        out.append(wrap(mixed << gain_shift, gain_bits))
        phase = (phase + PHASE_INC) & 0xFFFFFFFF
    return out


def stats(values):
    peak = max(abs(v) for v in values)
    active = sum(1 for v in values if v != 0)
    loud = sum(1 for v in values if abs(v) > 100000)
    return peak, active, loud


def main():
    with wave.open(WAV, "rb") as w:
        channels = w.getnchannels()
        width = w.getsampwidth()
        rate = w.getframerate()
        raw = w.readframes(w.getnframes())

    assert channels == 1 and width == 2, (channels, width)
    pcm = [v[0] for v in struct.iter_unpack("<h", raw)]
    bank = [encode(v) for v in pcm]

    with open(HEX, "w", encoding="ascii") as f:
        for code in bank:
            f.write("%02x\n" % code)

    print("file: %d samples, %d Hz, %d ch, %d bit"
          % (len(pcm), rate, channels, width * 8))
    print("bank words=%d, hex dumped to %s" % (len(bank), HEX))
    print("file peak pcm = %d, first loud slot = %d"
          % (max(abs(v) for v in pcm),
             next(i for i, c in enumerate(bank) if c != 0xFF)))

    exact = play(bank, FRAMES, len(bank), 7)
    trunc = play(bank, FRAMES, len(bank), 7, gain_bits=18)
    print("\nfirst %d frames:" % FRAMES)
    for name, values in (("fixed 24 bit", exact),
                         ("old 18 bit trunc", trunc)):
        peak, active, loud = stats(values)
        print("  %-18s peak=%9d active=%5d loud(>100000)=%5d"
              % (name, peak, active, loud))


if __name__ == "__main__":
    main()
