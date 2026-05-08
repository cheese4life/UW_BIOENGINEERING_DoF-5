#!/usr/bin/env python3
# oscillation for DOF to demonstrate how program interacts
# with the hardware on a script basis
# specific script only for Linux enviroments 

import os, sys, time, struct, termios, tty, select

import site

# pointing to can module
sys.path.insert(0, site.getusersitepackages())
import can



TOP_EDGE_MM = 1.0
BOTTOM_EDGE_MM = -1.0

CHANNEL = 'can0' # linux network interface name

# details fetched from PMD Juno Chip Datasheet
# refer to doc 41-1212 on shared drive

# format --> TX frame: [axis] [opcode] [parameters]
# ex)        TX bytes:   00      10     00 09 27 C0

TX_ID = 0x600 # CAN ID, listen to this address
RX_ID = 0x580 # CAN ID, respond to this address

AXIS = 0 # from Juno chip: which motor to talk to (zero indexed)


# encoding: 200000 pulses per mm, so position update every 5 nm
COUNTS_PER_MM = 200000 # used for human-readable conversion

# Juno chip clock cycle: 19608 Hz, every 51 micro seconds
# retrieved from test script requesting cycle information
SAMPLE_S = 51e-6 # used for converting sent units to clock cycle
"""
Mathematical Breakdwon
0.2 mm/s 
= 40,000 counts/s          (× COUNTS_PER_MM)
× 51e-6 s/cycle            (× SAMPLE_S)
= 2.04 counts/cycle        (what the chip actually wants)
× 65536                    (fixed-point scaling the chip uses internally)
= 133,693 = register value

40,000 * 0.000051 * 65,536 = 133,693 (register value)

Conversions in acc_reg() and vel_reg()
"""

SOFT_LIMIT_POS_MM = 1.2
SOFT_LIMIT_NEG_MM = -1.2


TICK_S = 0.02

# all opticodes (0x00) from juno chip documentation
# confirmed with test script
OP_SET_POSITION, OP_SET_VELOCITY, OP_UPDATE = 0x10, 0x11, 0x1A
OP_RESET_EVENT, OP_GET_ACT_POS = 0x34, 0x37
OP_SET_OPMODE, OP_SET_MOTOR_CMD = 0x65, 0x77
OP_SET_ACC, OP_SET_DEC = 0x90, 0x91
OP_CAL_ANALOG = 0xF5
OPMODE_CAL, OPMODE_FULL = 0x06, 0x37


def is_timeout_error(exc):
    return isinstance(exc, RuntimeError) and "timeout op=" in str(exc)

# this guy touches the CAN interface
# p = b'' converts p to bytes
# bus = open CAN interface object
# op = opticode command
def sr(bus, op, p=b'', timeout = 0.2):
    
    # bus - open CAN interface
    # can.Message() -> constructs CAN frame object
    # arbitration_id=TX_ID -> sets ID to Juno chip expectation
    # data=bytes([AXIS, op]) -> builds message payload. +p appends parameters:
        # result looks like: [0x00, 0x10, 0x00, 0x09, 0x27, 0xC0] (example)
    
    bus.send(can.Message(arbitration_id=TX_ID, data=bytes([AXIS, op]) + p, 
                         is_extended_id=False))
    
    # data recieve and timeout logic
    
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        m = bus.recv(timeout=end - time.monotonic())
        if m and m.arbitration_id == RX_ID:
            return bytes(m.data)
        # error
    raise RuntimeError(f"timeout op=0x{op:02X}")


    # decoder for recieved message
def s32(d):
    # RX frame: [status] [axis] [data bytes...]
    body = d[2:] # strips status and axis bytes
    if not body:
        return 0 # if empty
    
    # body[0] & 0x80 --> checks if negative
    # if negative, sign byte is 0xFF. Else, sign byte is 0x00
    # core piece that decodes "forward vs backward" or "pos/neg" position
    sign = b'\xff' if body[0] & 0x80 else b'\x00'
    
    # full breakdown of communications:
    """
    0x03  =  0000 0011
    0x80  =  1000 0000
            ---------
    AND  =   0000 0000  -> zero -> positive -> sign = b'\x00'
    
    """
    
    
    return struct.unpack('>i', sign * (4 - len(body)) + body)[0] # returns position encoder counts
    
def get_pos(bus):
    # sent to sr to be decoded
    return s32(sr(bus, OP_GET_ACT_POS))

# unit converter, sent to SetVelocity register
def vel_reg(mm_s):
                                # 65536 because chip stores velocity 
                                # as fixed-point number. (Juno Docs)
    return int(round(mm_s * COUNTS_PER_MM * SAMPLE_S * 65536))

# unit converter, sent to SetAcceleration register
def acc_reg(mm_s2):
    return int(round(mm_s2 * COUNTS_PER_MM * SAMPLE_S * SAMPLE_S * 65536))

# first-boot events
def init_drive(bus):
    print("Initializing drive...")
    
    # PLEASE refer to Juno Chip docs to better understand where all this comes from
    
    # (0x34) clears motion error + instruction error latched bits
    sr(bus, OP_RESET_EVENT, struct.pack('>H', 0xA000))
    
    # (0x34) clears all remaining event status bits
    sr(bus, OP_RESET_EVENT, struct.pack('>H', 0xEFFF))
    
    # (0x77) sets motor output to zero before mode switch
    sr(bus, OP_SET_MOTOR_CMD, struct.pack('>h', 0))
    
    # (0x65) puts chip in callibration mode 
    sr(bus, OP_SET_OPMODE, struct.pack('>H', OPMODE_CAL))
    
    
    time.sleep(0.05)
    
    # (0xF5) measures current sensor offsets. 
    #        Accounts for drift during powered-off session
    sr(bus, OP_CAL_ANALOG, struct.pack('>H', 0))
    
    # sleep for callibration cycle (~200ms)
    time.sleep(0.2)
    
    # (0x34) clears any events raised during callibration
    sr(bus, OP_RESET_EVENT, struct.pack('>H', 0xEFFF))
    
    # (0x65) enables full servo
    sr(bus, OP_SET_OPMODE, struct.pack('>H', OPMODE_FULL))
    time.sleep(0.05)
    
    print(f"  servo on at {get_pos(bus)/COUNTS_PER_MM:+.4f} mm")


def cmd_target(bus, target_counts, vel_mm_s, acc_mm_s2):
    # sets accel register. converts units to register value
    sr(bus, OP_SET_ACC, struct.pack('>i', acc_reg(acc_mm_s2)))
    
    # sets deceleration
    sr(bus, OP_SET_DEC, struct.pack('>i', acc_reg(acc_mm_s2)))
    
    # sets velocity limit, DOF won't exceed this limit
    sr(bus, OP_SET_VELOCITY, struct.pack('>i', vel_reg(vel_mm_s)))
    
    # sets destination position in encoder counts
    sr(bus, OP_SET_POSITION, struct.pack('>i', target_counts))
    
    # sets destination
    sr(bus, OP_UPDATE)

def read_keys_nonblock(fd, pending):
    # drain whatever bytes are ready on stdin (non-blocking)
    while select.select([fd], [], [], 0)[0]:
        chunk = os.read(fd, 64)
        if not chunk:
            break
        pending += chunk
    keys = []
    while pending:
        b = pending[0]
        pending = pending[1:]
        if b == 0x03:
            keys.append('CTRL_C')
        elif b == 0x1B:
            pending = b''  # discard escape sequences (arrow keys etc.)
        else:
            keys.append(chr(b))
    return keys, pending


def clamp_soft_limit(pos_mm):
    if pos_mm > SOFT_LIMIT_POS_MM:
        return SOFT_LIMIT_POS_MM
    if pos_mm < SOFT_LIMIT_NEG_MM:
        return SOFT_LIMIT_NEG_MM
    return pos_mm

def main():
    # state machine
    # GOING_POS: stage is moving toward top_edge_mm
    # GOING_NEG: stage is moving toward bottom_edge_mm

    # socketcan is linux kernals built-in CAN bus.
    # exposes CAN interfaces like can0
    bus = can.interface.Bus(channel=CHANNEL, interface='socketcan')

    vel_mm_s = 1.0
    acc_mm_s2 = 20.0

    try:                                          # outer try — guarantees bus.shutdown()
        init_drive(bus)
        print(f"Oscillating between {BOTTOM_EDGE_MM:+.2f} and {TOP_EDGE_MM:+.2f} mm at {vel_mm_s} mm/s")
        print("press 'q' to quit")

        # gets file descriptor number for standard input (keyboard)
        fd = sys.stdin.fileno()

        # saves current terminal settings.
        # Restores terminal after you press q
        old = termios.tcgetattr(fd)
        # switches terminal to cbreak mode (keystrokes delivered to program)
        tty.setcbreak(fd)

        try:                                     
            # initial state of state machine
            direction = +1
            quit_flag = False
            # for SSH
            pending = b''

            # bootstrap. fire first move before loop starts
            cmd_target(bus, int(round(TOP_EDGE_MM * COUNTS_PER_MM)), vel_mm_s, acc_mm_s2)

            while not quit_flag:
                try:                              # per-tick try, catches CAN timeouts
                    keys, pending = read_keys_nonblock(fd, pending)
                    for k in keys:
                        if k in ('q', 'Q', 'CTRL_C'):
                            quit_flag = True
                    if quit_flag:
                        break

                    pos_mm = get_pos(bus) / COUNTS_PER_MM

                    if direction == +1 and pos_mm >= TOP_EDGE_MM:
                        direction = -1
                        cmd_target(bus, int(round(BOTTOM_EDGE_MM * COUNTS_PER_MM)), vel_mm_s, acc_mm_s2)
                        sys.stdout.write(f"\r  flip -> NEG  pos {pos_mm:+.4f} mm    \n")
                        sys.stdout.flush()
                    elif direction == -1 and pos_mm <= BOTTOM_EDGE_MM:
                        direction = +1
                        cmd_target(bus, int(round(TOP_EDGE_MM * COUNTS_PER_MM)), vel_mm_s, acc_mm_s2)
                        sys.stdout.write(f"\r  flip -> POS  pos {pos_mm:+.4f} mm    \n")
                        sys.stdout.flush()
                    else:
                        sys.stdout.write(f"\r  {'>>>' if direction == +1 else '<<<'}  pos {pos_mm:+.4f} mm    ")
                        sys.stdout.flush()

                    time.sleep(TICK_S)

                except RuntimeError as e:
                    if not is_timeout_error(e):
                        raise
                    print("\n[timeout] CAN timeout — is the stage powered on?")
                    quit_flag = True

        finally:
            # always restore terminal, even if an exception escaped the loop
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    finally:
        # always close the CAN bus, even on crash
        bus.shutdown()


if __name__ == '__main__':
    main()
















