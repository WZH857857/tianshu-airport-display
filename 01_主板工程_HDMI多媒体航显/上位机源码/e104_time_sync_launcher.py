"""Launch the unchanged E104 console with transparent HDMI time and alarm sync.

The E104 console files are not edited or replaced.  This launcher imports the
same application, wraps its existing once-per-second clock tick, and forwards
two things to the HDMI board over the main serial link:

  * the RTC time shown by the console (command 0x22, ADV20 and later);
  * the latched flame alarm of the emergency page (command 0x23, ADV21 and
    later), so the HDMI board switches to the fire evacuation screen and
    sounds its own buzzer while the alarm is active.

The alarm forwarder follows the console's own main-board link while it is
open.  When that link is closed - the usual state when nobody pressed
"连接" - the frame is pushed through a short direct serial session on the
board port instead of being dropped, so "危险已解除" always reaches the
HDMI board.
"""

from __future__ import annotations

import os
import sys
import time
from types import SimpleNamespace


E104_DIR = os.environ.get(
    "E104_CONSOLE_DIR",
    r"E:\FPGA guo sai\EG4X20B\E104_Console",
)
# Firmware versions the launch wrapper expects from the HDMI board: 0x1A is
# the first build with the TIME command, 0x1B the first one with the fire
# emergency command.
TIME_MIN_VERSION = 0x1A
EMG_MIN_VERSION = 0x1B
CMD_TIME = 0x22
CMD_EMG = 0x23
EMG_RESYNC_TICKS = 5

# Direct fallback link for the alarm command (same framing as the board).
BOARD_BAUD = 115200
BOARD_PORT_ENV = "HDMI_BOARD_PORT"
DIRECT_RESYNC_MIN_GAP = 15.0
DIRECT_CLEAR_MIN_GAP = 2.0
HDR1 = 0xA5
HDR2 = 0x5A
CMD_PING = 0x01
R_PING = 0x81
R_EMG = 0xA3

# Timestamp of the last direct serial session, and of the last failure written
# to the log.  Re-opening the port is rate limited so an idle alarm cannot
# hammer a board that is switched off, and the log keeps one line per quarter
# minute instead of one per second.
_direct_try: dict = {"t": 0.0}
_direct_log: dict = {"t": 0.0}

# Last flame state pushed to the HDMI board.  None means "nothing sent yet".
_emg_last: dict = {"fire": None, "tick": 0}
# The console can keep reporting the old latched alarm for a moment after the
# clear button was pressed.  Suppress that stale re-arm for a short window so
# the HDMI board does not bounce straight back into the emergency screen.  A
# genuinely new alarm is forwarded again once the window expires.
_emg_clear_hold: dict = {"until": 0.0}
_emg_forward_log = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "emg_forward.log"
)


def _log_emg_forward(fire: int, force: bool, override, via: str) -> None:
    try:
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        with open(_emg_forward_log, "a", encoding="utf-8") as handle:
            handle.write("%s fire=%d force=%s override=%s via=%s\n"
                         % (stamp, fire, force, override, via))
    except Exception:
        pass


def _board_frame(cmd: int, payload: bytes = b"") -> bytes:
    body = bytes((cmd & 0xFF, len(payload) & 0xFF)) + bytes(payload)
    checksum = 0
    for byte in body:
        checksum ^= byte
    return bytes((HDR1, HDR2)) + body + bytes((checksum,))


def _board_read_frame(port, timeout: float = 0.6):
    """Return (cmd, payload) of the next valid board frame, or None."""
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = port.read(64)
        if chunk:
            buf.extend(chunk)
        while len(buf) >= 4:
            if buf[0] != HDR1 or buf[1] != HDR2:
                del buf[0]
                continue
            length = buf[3]
            total = 5 + length
            if len(buf) < total:
                break
            raw = bytes(buf[:total])
            del buf[:total]
            checksum = 0
            for byte in raw[2:4 + length]:
                checksum ^= byte
            if checksum == raw[4 + length]:
                return raw[2], raw[4:4 + length]
    return None


def _board_port_candidates() -> list:
    """Board port candidates, best guess first."""
    explicit = os.environ.get(BOARD_PORT_ENV)
    if explicit:
        return [explicit]
    names = []
    try:
        from serial.tools import list_ports

        for info in list_ports.comports():
            desc = (getattr(info, "description", "") or "").lower()
            if "ch340" in desc or "usb-serial" in desc or "usb serial" in desc:
                names.append(info.device)
    except Exception:
        pass
    for fallback in ("COM6",):
        if fallback not in names:
            names.append(fallback)
    return names


def _direct_send_emg(fire: int) -> str | None:
    """Send command 0x23 on a short-lived serial session of our own.

    Returns the port that accepted the frame, or None when no candidate
    answered.
    """
    try:
        import serial
    except Exception:
        return None

    for name in _board_port_candidates():
        try:
            with serial.Serial(name, BOARD_BAUD, timeout=0.15,
                               write_timeout=0.5) as port:
                time.sleep(0.05)
                port.reset_input_buffer()

                port.write(_board_frame(CMD_PING))
                reply = _board_read_frame(port)
                if reply is None or reply[0] != R_PING or not reply[1]:
                    continue
                # A board below 0x1B does not know command 0x23.
                if reply[1][0] < EMG_MIN_VERSION:
                    continue

                port.write(_board_frame(CMD_EMG, bytes((fire,))))
                ack = _board_read_frame(port)
                if ack is not None and ack[0] == R_EMG:
                    return name
        except Exception:
            continue
    return None

if os.path.isdir(E104_DIR) and E104_DIR not in sys.path:
    sys.path.insert(0, E104_DIR)

try:
    import e104_console as e104
except Exception as exc:  # pragma: no cover - depends on the local install
    raise SystemExit("Cannot import the E104 console from %s: %s" % (E104_DIR, exc))


ac = e104.ac
_original_tick_clock = ac.Console._tick_clock


def _bcd_to_int(value) -> int | None:
    if not isinstance(value, int) or isinstance(value, bool):
        return None
    if value < 0 or value > 0xFF:
        return None
    high = (value >> 4) & 0x0F
    low = value & 0x0F
    if high > 9 or low > 9:
        return None
    return high * 10 + low


def _rtc_payload(status) -> bytes | None:
    """Return the exact BCD fields displayed by the E104 RTC card."""
    if not isinstance(status, dict):
        return None
    if "rtc_valid" in status and not status.get("rtc_valid"):
        return None

    hour = _bcd_to_int(status.get("rtc_h"))
    minute = _bcd_to_int(status.get("rtc_m"))
    second = _bcd_to_int(status.get("rtc_s"))
    if hour is None or minute is None or second is None:
        return None
    if not (0 <= hour <= 23 and 0 <= minute <= 59 and 0 <= second <= 59):
        return None

    return bytes(
        (
            status["rtc_h"],
            status["rtc_m"],
            status["rtc_s"],
        )
    )


def _console_rtc_payload(self) -> bytes | None:
    # The visible "实时时钟" card is fed by the emergency/Bluetooth status.
    # Only use it while that link is live; otherwise stale card data would
    # repeatedly rewind the HDMI clock.
    if getattr(self, "_emg_connected", False):
        payload = _rtc_payload(getattr(self, "_emg_status", None))
        if payload is not None:
            return payload

    # Some builds also include the DS1307 fields in the main-board status.
    return _rtc_payload(getattr(self, "_status", None))


def _tick_clock_with_hdmi_time(self) -> None:
    _original_tick_clock(self)

    worker = getattr(self, "worker", None)
    if worker is None or not getattr(self, "connected", False):
        return

    status = getattr(self, "_status", None)
    if not isinstance(status, dict):
        return
    version = status.get("ver")
    if not isinstance(version, int) or version < TIME_MIN_VERSION:
        return

    payload = _console_rtc_payload(self)
    if payload is not None:
        try:
            worker.send(CMD_TIME, payload)
        except Exception:
            # The console already reports link failures; time sync must never
            # become a second source of popups or thread errors.
            pass

    # A board reset drops the emergency latch, so the state is pushed again
    # every few seconds even when the E104 side has not changed.
    _emg_last["tick"] += 1
    _forward_emergency(self, force=(_emg_last["tick"] % EMG_RESYNC_TICKS == 0))


def _board_fire(self):
    """Return 0/1 for the flame alarm of the emergency link, None if unknown."""
    if not getattr(self, "_emg_connected", False):
        return None
    status = getattr(self, "_emg_status", None)
    if not isinstance(status, dict):
        return None
    if "flame_alarm" not in status:
        return None
    return 1 if status.get("flame_alarm") else 0


def _forward_emergency(self, force: bool = False,
                       fire_override: int | None = None) -> None:
    """Mirror the E104 flame alarm onto the HDMI board (command 0x23)."""
    worker = getattr(self, "worker", None)
    linked = worker is not None and getattr(self, "connected", False)

    board = getattr(self, "_status", None)
    version = board.get("ver") if isinstance(board, dict) else None
    if linked and (not isinstance(version, int) or version < EMG_MIN_VERSION):
        # 0x1B is the first firmware that understands the EMG command.  A
        # board that still reports 0x1A must never receive an unknown frame.
        return

    if fire_override is None:
        fire = _board_fire(self)
        if fire == 1 and time.monotonic() < _emg_clear_hold["until"]:
            fire = 0
        elif fire == 0:
            _emg_clear_hold["until"] = 0.0
    else:
        fire = 1 if fire_override else 0
    if fire is None:
        return

    changed = fire != _emg_last["fire"]
    if not force and not changed:
        return

    if linked:
        try:
            worker.send(CMD_EMG, bytes((fire,)))
        except Exception:
            # The console already reports link failures; the alarm forwarder
            # must not become a second source of popups or thread errors.
            return
        via = "link"
    else:
        # The console's main link is closed (or was never opened), so the
        # frame goes straight onto the board port instead of being dropped.
        # A changed state and the user's clear button always go out at once;
        # background re-syncs are rate limited.
        now = time.monotonic()
        gap = (DIRECT_CLEAR_MIN_GAP if (changed or fire_override is not None)
               else DIRECT_RESYNC_MIN_GAP)
        # The user's own clear button is never delayed.
        if fire_override is None and now - _direct_try["t"] < gap:
            return
        _direct_try["t"] = now
        port_used = _direct_send_emg(fire)
        if port_used is None:
            # Leave the state untouched so the next tick retries, and keep a
            # trace: an empty log is otherwise indistinguishable from a click
            # that never happened.
            if now - _direct_log["t"] >= DIRECT_RESYNC_MIN_GAP:
                _direct_log["t"] = now
                _log_emg_forward(fire, force, fire_override, "direct-failed")
            return
        via = "direct:" + str(port_used)

    _emg_last["fire"] = fire
    _log_emg_forward(fire, force, fire_override, via)


_original_e104_send_clear = getattr(ac.Console, "_e104_send_clear", None)


def _e104_send_clear_with_hdmi_clear(self) -> None:
    """Clear the HDMI alarm as soon as the console's clear button is pressed."""
    if _original_e104_send_clear is not None:
        _original_e104_send_clear(self)
    # The E104 clear command only unlocks its own latched alarm.  The HDMI
    # board keeps its separate 0x23 latch, so release it explicitly too.  This
    # must work even if the Bluetooth link has just dropped, because the user
    # has already decided that the alarm is over.
    _emg_clear_hold["until"] = time.monotonic() + 5.0
    _forward_emergency(self, force=True, fire_override=0)


if _original_e104_send_clear is not None:
    ac.Console._e104_send_clear = _e104_send_clear_with_hdmi_clear


_original_emg_on_status = getattr(ac.Console, "_emg_on_status", None)


def _emg_on_status_with_forward(self, status) -> None:
    """Push the alarm as soon as a Bluetooth status frame arrives."""
    if _original_emg_on_status is not None:
        _original_emg_on_status(self, status)
    _forward_emergency(self)


if _original_emg_on_status is not None:
    ac.Console._emg_on_status = _emg_on_status_with_forward


_original_emg_on_reply = getattr(ac.Console, "_emg_on_reply", None)


def _emg_on_reply_with_hdmi_clear(self, cmd, payload) -> None:
    if _original_emg_on_reply is not None:
        _original_emg_on_reply(self, cmd, payload)
    if cmd == getattr(e104, "R_CLEAR", 0x91):
        _emg_clear_hold["until"] = time.monotonic() + 5.0
        _forward_emergency(self, force=True, fire_override=0)


if _original_emg_on_reply is not None:
    ac.Console._emg_on_reply = _emg_on_reply_with_hdmi_clear


ac.Console._tick_clock = _tick_clock_with_hdmi_time


def _selftest_body() -> int:
    assert _rtc_payload(
        {"rtc_valid": 1, "rtc_h": 0x01, "rtc_m": 0x18, "rtc_s": 0x05}
    ) == b"\x01\x18\x05"
    assert _rtc_payload(
        {"rtc_valid": 0, "rtc_h": 0x01, "rtc_m": 0x18, "rtc_s": 0x05}
    ) is None
    assert _rtc_payload(
        {"rtc_valid": 1, "rtc_h": 0x2A, "rtc_m": 0x18, "rtc_s": 0x05}
    ) is None
    assert _rtc_payload(
        {"rtc_valid": 1, "rtc_h": 0x23, "rtc_m": 0x59, "rtc_s": 0x59}
    ) == b"\x23\x59\x59"
    assert _rtc_payload(
        {"rtc_valid": 1, "rtc_h": 0x24, "rtc_m": 0x00, "rtc_s": 0x00}
    ) is None

    live = SimpleNamespace(
        _emg_connected=True,
        _emg_status={"rtc_valid": 1, "rtc_h": 0x01, "rtc_m": 0x18, "rtc_s": 0x05},
        _status={"rtc_h": 0x02, "rtc_m": 0x18, "rtc_s": 0x05},
    )
    assert _console_rtc_payload(live) == b"\x01\x18\x05"

    main_only = SimpleNamespace(
        _emg_connected=False,
        _emg_status={"rtc_valid": 1, "rtc_h": 0x01, "rtc_m": 0x18, "rtc_s": 0x05},
        _status={"rtc_h": 0x02, "rtc_m": 0x18, "rtc_s": 0x05},
    )
    assert _console_rtc_payload(main_only) == b"\x02\x18\x05"

    class FakeWorker:
        def __init__(self):
            self.frames = []

        def send(self, cmd, payload=b""):
            self.frames.append((cmd, bytes(payload)))

    worker = FakeWorker()
    console = SimpleNamespace(
        worker=worker,
        connected=True,
        _status={"ver": EMG_MIN_VERSION},
        _emg_connected=True,
        _emg_status={"flame_alarm": 1},
    )
    _emg_last["fire"] = None
    _forward_emergency(console)
    assert worker.frames == [(CMD_EMG, b"\x01")]
    _forward_emergency(console)                 # unchanged -> no repeat
    assert worker.frames == [(CMD_EMG, b"\x01")]
    _forward_emergency(console, force=True)     # resync still sends
    assert worker.frames == [(CMD_EMG, b"\x01"), (CMD_EMG, b"\x01")]
    console._emg_status = {"flame_alarm": 0}
    _forward_emergency(console)
    assert worker.frames[-1] == (CMD_EMG, b"\x00")

    worker.frames.clear()
    console._status = {"ver": 0x1A}             # ADV20 board: no 0x23
    console._emg_status = {"flame_alarm": 1}
    _forward_emergency(console)
    assert worker.frames == []

    console._status = {"ver": EMG_MIN_VERSION}
    console._emg_connected = False              # Bluetooth down
    _forward_emergency(console)
    assert worker.frames == []

    # The visible clear button must unlock the HDMI board even when the
    # Bluetooth link is already down or its status page has no alarm field.
    _forward_emergency(console, force=True, fire_override=0)
    assert worker.frames[-1] == (CMD_EMG, b"\x00")

    # With the console's own main link closed the frame must still go out
    # through a short direct serial session on the board port.
    direct_sent = []
    saved_direct = globals()["_direct_send_emg"]
    globals()["_direct_send_emg"] = (
        lambda fire: (direct_sent.append(fire), "COM7")[1]
    )
    try:
        console.worker = None
        console.connected = False
        console._emg_connected = True
        console._emg_status = {"flame_alarm": 1}
        _emg_last["fire"] = None
        _direct_try["t"] = 0.0
        _direct_log["t"] = 0.0
        _forward_emergency(console)
        assert direct_sent == [1], direct_sent
        _forward_emergency(console, force=True)      # rate limited
        assert direct_sent == [1], direct_sent
        _direct_try["t"] = 0.0                       # a couple of seconds later
        console._emg_status = {"flame_alarm": 0}
        _forward_emergency(console, force=True)
        assert direct_sent == [1, 0], direct_sent
        # "危险已解除" goes out immediately even while the state was just 0.
        _forward_emergency(console, force=True, fire_override=0)
        assert direct_sent == [1, 0, 0], direct_sent

        # A board that is switched off must not be recorded as delivered.
        globals()["_direct_send_emg"] = lambda fire: None
        _emg_last["fire"] = None
        _direct_try["t"] = 0.0
        _direct_log["t"] = 0.0
        _forward_emergency(console, force=True)
        assert _emg_last["fire"] is None, _emg_last
    finally:
        globals()["_direct_send_emg"] = saved_direct

    print("PASS: E104 RTC time forwarder selftest")
    return 0


def _selftest() -> int:
    """Run the checks without writing selftest traffic into the forward log."""
    global _emg_forward_log
    saved_log = _emg_forward_log
    _emg_forward_log = os.devnull
    try:
        return _selftest_body()
    finally:
        _emg_forward_log = saved_log


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        raise SystemExit(_selftest())
    sys.exit(ac.main())
