#!/usr/bin/env python3
"""Continuous arrow-key jog for DOF-5.

Hold Up/Down arrow -> stage moves continuously in that direction.
Release the key      -> stage decelerates and holds.

Other keys:
    [ / ]   : decrease / increase jog speed
    h       : print current position
    q       : quit (servo keeps holding position)
    Ctrl-C  : quit

How it works: terminal key-repeat sends the arrow keystroke ~30x/sec while held.
We extend an absolute position target ~0.4 mm ahead of the live encoder reading
on each tick. When ~120 ms passes with no arrow key, we brake by commanding the
current position as the target.

Power-loss behavior:
If stage power is cut, CAN reads time out (for example timeout op=0x37 on
GetActualPosition). The script treats this as a likely power-loss event,
waits for the stage to come back, re-initializes the drive, then resumes.
"""
import os, sys, struct, time, termios, tty, select, site
sys.path.insert(0, site.getusersitepackages())
import can  # type: ignore

CHANNEL, TX_ID, RX_ID, AXIS = 'can0', 0x600, 0x580, 0
COUNTS_PER_MM = 200_000
SAMPLE_S = 51e-6
# Asymmetric software travel limits in mm.
# Positive side is tighter because the stage beeps near +1.8 mm.
SOFT_LIMIT_POS_MM = 1.6
SOFT_LIMIT_NEG_MM = -2.4
LOOKAHEAD_MM = 0.4
RELEASE_TIMEOUT_S = 0.12
TICK_S = 0.02

OP_SET_POSITION, OP_SET_VELOCITY, OP_UPDATE = 0x10, 0x11, 0x1A
OP_RESET_EVENT, OP_GET_ACT_POS = 0x34, 0x37
OP_SET_OPMODE, OP_SET_MOTOR_CMD = 0x65, 0x77
OP_SET_ACC, OP_SET_DEC = 0x90, 0x91
OP_CAL_ANALOG = 0xF5
OPMODE_CAL, OPMODE_FULL = 0x06, 0x37


def is_timeout_error(exc):
    return isinstance(exc, RuntimeError) and "timeout op=" in str(exc)


def sr(bus, op, p=b'', timeout=0.2):
    bus.send(can.Message(arbitration_id=TX_ID, data=bytes([AXIS, op]) + p,
                         is_extended_id=False))
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        m = bus.recv(timeout=end - time.monotonic())
        if m and m.arbitration_id == RX_ID:
            return bytes(m.data)
    raise RuntimeError(f"timeout op=0x{op:02X}")


def s32(d):
    body = d[2:]
    if not body:
        return 0
    sign = b'\xff' if body[0] & 0x80 else b'\x00'
    return struct.unpack('>i', sign * (4 - len(body)) + body)[0]


def get_pos(bus):
    return s32(sr(bus, OP_GET_ACT_POS))


def vel_reg(mm_s):
    return int(round(mm_s * COUNTS_PER_MM * SAMPLE_S * 65536))


def acc_reg(mm_s2):
    return int(round(mm_s2 * COUNTS_PER_MM * SAMPLE_S * SAMPLE_S * 65536))


def init_drive(bus):
    print("Initializing drive...")
    sr(bus, OP_RESET_EVENT, struct.pack('>H', 0xA000))
    sr(bus, OP_RESET_EVENT, struct.pack('>H', 0xEFFF))
    sr(bus, OP_SET_MOTOR_CMD, struct.pack('>h', 0))
    sr(bus, OP_SET_OPMODE, struct.pack('>H', OPMODE_CAL))
    time.sleep(0.05)
    sr(bus, OP_CAL_ANALOG, struct.pack('>H', 0))
    time.sleep(0.2)
    sr(bus, OP_RESET_EVENT, struct.pack('>H', 0xEFFF))
    sr(bus, OP_SET_OPMODE, struct.pack('>H', OPMODE_FULL))
    time.sleep(0.05)
    print(f"  servo on at {get_pos(bus)/COUNTS_PER_MM:+.4f} mm")


def cmd_target(bus, target_counts, vel_mm_s, acc_mm_s2):
    sr(bus, OP_SET_ACC, struct.pack('>i', acc_reg(acc_mm_s2)))
    sr(bus, OP_SET_DEC, struct.pack('>i', acc_reg(acc_mm_s2)))
    sr(bus, OP_SET_VELOCITY, struct.pack('>i', vel_reg(vel_mm_s)))
    sr(bus, OP_SET_POSITION, struct.pack('>i', target_counts))
    sr(bus, OP_UPDATE)


def clamp_soft_limit(pos_mm):
    if pos_mm > SOFT_LIMIT_POS_MM:
        return SOFT_LIMIT_POS_MM
    if pos_mm < SOFT_LIMIT_NEG_MM:
        return SOFT_LIMIT_NEG_MM
    return pos_mm


def read_keys_nonblock(fd, pending):
    """Return parsed keys plus pending bytes for next tick.

    Remote terminal sessions can split arrow-key sequences across packets
    (ESC then [A later). Keeping `pending` prevents accidental ESC handling.
    """
    while select.select([fd], [], [], 0)[0]:
        chunk = os.read(fd, 64)
        if not chunk:
            break
        pending += chunk

    keys = []
    while pending:
        b0 = pending[0]

        # Normal one-byte key.
        if b0 != 0x1B:
            pending = pending[1:]
            if b0 == 0x03:
                keys.append('CTRL_C')
            else:
                keys.append(chr(b0))
            continue

        # ESC-prefixed sequence.
        if len(pending) == 1:
            # Bare ESC: ignore instead of quitting.
            pending = b''
            break

        if pending[1] in (ord('['), ord('O')):
            # Parse CSI/SS3 until final byte in 0x40..0x7E.
            i = 2
            while i < len(pending) and not (0x40 <= pending[i] <= 0x7E):
                i += 1
            if i >= len(pending):
                # Incomplete sequence, wait for next loop tick.
                break

            final = chr(pending[i])
            pending = pending[i + 1:]
            if final == 'A':
                keys.append('UP')
            elif final == 'B':
                keys.append('DOWN')
            elif final == 'C':
                keys.append('RIGHT')
            elif final == 'D':
                keys.append('LEFT')
            continue

        # Unknown ESC sequence (e.g. Alt+key): drop ESC and continue.
        pending = pending[1:]

    return keys, pending


def wait_for_stage_recovery(bus, fd, pending):
    """Wait for stage comms to return, then re-run init sequence.

    Returns (recovered, pending).
    """
    print("\n[power-loss] Stage timeout detected (likely power cut).")
    print("[power-loss] Waiting for stage power to return... (press q to quit)")

    next_msg = 0.0
    while True:
        keys, pending = read_keys_nonblock(fd, pending)
        for k in keys:
            if k in ('q', 'Q', 'CTRL_C'):
                print("[power-loss] Recovery cancelled by user.")
                return False, pending

        try:
            _ = get_pos(bus)  # probe comms
            print("[power-loss] Stage responding again. Re-initializing drive...")
            init_drive(bus)
            print("[power-loss] Recovery complete. Continuing jog loop.")
            return True, pending
        except RuntimeError as e:
            if not is_timeout_error(e):
                raise

        now = time.monotonic()
        if now >= next_msg:
            print("[power-loss] still waiting for CAN response...")
            next_msg = now + 2.0
        time.sleep(0.2)


def main():
    bus = can.interface.Bus(channel=CHANNEL, interface='socketcan')
    vel_mm_s = 2.0
    acc_mm_s2 = 50.0
    try:
        init_drive(bus)
        print()
        print("  Hold Up/Down arrow to jog continuously.  [/] vel  h pos  q quit")
        print(
            f"  vel={vel_mm_s:.2f} mm/s   soft limits "
            f"[{SOFT_LIMIT_NEG_MM:+.2f}, {SOFT_LIMIT_POS_MM:+.2f}] mm"
        )
        print()

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            direction = 0
            last_arrow_t = 0.0
            quit_flag = False
            pending = b''
            while not quit_flag:
                try:
                    keys, pending = read_keys_nonblock(fd, pending)
                    for k in keys:
                        if k == 'UP':
                            direction = +1; last_arrow_t = time.monotonic()
                        elif k == 'DOWN':
                            direction = -1; last_arrow_t = time.monotonic()
                        elif k in ('q', 'Q', 'CTRL_C'):
                            quit_flag = True
                        elif k == ']':
                            vel_mm_s = min(vel_mm_s * 1.5, 20.0)
                            sys.stdout.write(f"\r  vel = {vel_mm_s:.2f} mm/s          \n")
                        elif k == '[':
                            vel_mm_s = max(vel_mm_s / 1.5, 0.05)
                            sys.stdout.write(f"\r  vel = {vel_mm_s:.2f} mm/s          \n")
                        elif k in ('h', 'H'):
                            sys.stdout.write(f"\r  pos = {get_pos(bus)/COUNTS_PER_MM:+.4f} mm        \n")

                    if quit_flag:
                        break

                    pos = get_pos(bus)
                    pos_mm = pos / COUNTS_PER_MM

                    if direction != 0 and (time.monotonic() - last_arrow_t) < RELEASE_TIMEOUT_S:
                        target_mm = pos_mm + direction * LOOKAHEAD_MM
                        clamped = clamp_soft_limit(target_mm)
                        if clamped != target_mm:
                            target_mm = clamped
                            cmd_target(bus, int(round(target_mm * COUNTS_PER_MM)),
                                       vel_mm_s, acc_mm_s2)
                            sys.stdout.write(f"\r  ! soft limit at {target_mm:+.4f} mm    \n")
                            direction = 0
                        else:
                            cmd_target(bus, int(round(target_mm * COUNTS_PER_MM)),
                                       vel_mm_s, acc_mm_s2)
                        sys.stdout.write(
                            f"\r  jog {'+' if direction > 0 else '-'}  pos {pos_mm:+.4f} mm    ")
                        sys.stdout.flush()
                    elif direction != 0:
                        cmd_target(bus, pos, vel_mm_s, acc_mm_s2)
                        sys.stdout.write(f"\r  stopped at {pos_mm:+.4f} mm           \n")
                        sys.stdout.flush()
                        direction = 0

                    time.sleep(TICK_S)
                except RuntimeError as e:
                    if not is_timeout_error(e):
                        raise
                    direction = 0
                    recovered, pending = wait_for_stage_recovery(bus, fd, pending)
                    if not recovered:
                        quit_flag = True
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

        try:
            pos = get_pos(bus)
            cmd_target(bus, pos, vel_mm_s, acc_mm_s2)
            print(f"\nServo holding at {pos/COUNTS_PER_MM:+.4f} mm")
        except RuntimeError as e:
            if is_timeout_error(e):
                print("\nStage not responding at exit (likely power off), skipping hold command.")
            else:
                raise
    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
