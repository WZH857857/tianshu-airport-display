"""Generate the short 8 kHz mono WAV used by the ADV09 repeat testbench.

The real VOICE.WAV is about two minutes long, which is far too slow to push
through a behavioural simulation twice.  The repeat test needs a file that is
short enough to drain completely inside the shortened gap, so this script
writes a 25 ms, 8 kHz, 16 bit mono tone with the same header layout as the
real announcement.
"""

import math
import struct
import sys
from pathlib import Path


def main() -> int:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("sim/adv09_short.wav")
    rate = 8000
    frames = 200
    tone_hz = 440.0
    amplitude = 12000

    samples = bytearray()
    for n in range(frames):
        value = int(amplitude * math.sin(2.0 * math.pi * tone_hz * n / rate))
        samples += struct.pack("<h", value)

    data_size = len(samples)
    header = b"RIFF" + struct.pack("<I", 36 + data_size) + b"WAVE"
    header += b"fmt " + struct.pack("<IHHIIHH", 16, 1, 1, rate,
                                    rate * 2, 2, 16)
    header += b"data" + struct.pack("<I", data_size)

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(header + samples)
    print(f"wrote {out} ({len(header) + data_size} bytes, {frames} samples)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
