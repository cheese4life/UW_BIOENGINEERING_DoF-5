# NEED TO SET UP A HEADLESS LINUX BOX TO COMMUNICATE WITH THE DOF
#
# =============================================================================
# control.py - Control law for the cornea focus loop
# =============================================================================
#
# WHAT THIS FILE DOES (and what it does NOT)
# -----------------------------------------------------------------------------
# This file implements the *control law*: given a measured surface position
# (in pixels) and a focus target (also in pixels), it produces a stage move
# command in DOF stage units.
#
# It is PURE LOGIC. It knows nothing about CAN, USB, MotionSynergyAPI, or any
# operating system specifics. All hardware talk lives in dof_driver.py.
#
# Pipeline view:
#
#   surface.py  -->  control.py  -->  dof_driver.py  -->  DOF-5 stage
#   (where is      (how much should  (actually send       (physical motion)
#    the cornea?)   the stage move?)  the move command)
#
# Responsibilities here:
#   - pixel -> micrometer -> DOF unit conversion (config-driven)
#   - low-pass filter on the measurement (kills +/- 1 px jitter)
#   - deadband (don't move if error is within tolerance)
#   - max-move-per-frame safety cap (clipping)
#   - sign convention (which way is "up" for the stage vs. the image)
#
# Stateful: keeps the filtered position between frames (e.g. EMA).
#
# =============================================================================
# DOF-5 HARDWARE SUMMARY (from README.md)
# -----------------------------------------------------------------------------
# Stage:      Dover Motion DOF-5 Objective Focuser
# Travel:     5 mm (hard stop 6 mm); adjustable hard stops down to +/- 1.7 mm
# Encoder:    5 nm (E38, 200,000 cts/mm) or 1.25 nm (E39, 800,000 cts/mm)
# Resolution: 15 nm minimum move; 1 um bi-dir repeatability (2 sigma)
# Settle:     <= 15 ms for 100-250 nm moves (compatible with our 60 Hz loop)
# Bandwidth:  > 225 Hz servo
# Power:      24 VDC, both +VP (motor) and +VL (logic) must be powered
# Safety:     ALL MOVES ARE ABSOLUTE in Pro-Motion; relative moves are
#             implemented client-side as (current_position + delta).
# Required:   Stage MUST be homed after every power-on before position moves.
#
# Communication options on the stage side:
#   RS-232 (default 57600 8N1, point-to-point, addr 0)
#   RS-485 (2-wire or 4-wire, same defaults)
#   CAN 2.0B (factory baud 1 Mbit, Pro-Motion default 20 kbit, node id 0)
#   Step & Direction (TTL, up to 4.8 MHz) -- not useful for closed-loop here
#
# Our planned link: CAN via IXXAT USB-to-CAN V2 compact (kit 36102-00).
#
# =============================================================================
# MotionSynergyAPI ASSESSMENT (the host-OS problem)
# -----------------------------------------------------------------------------
# The MotionSynergyAPI shipped at:
#     /Users/antonbloch/Desktop/ivan lab/MotionSynergyAPI+Docs/
#         MotionSynergyAPI_4.0.16063_22.04/
#
# is a .NET library (MotionSynergyAPI.dll, MotionSynergyAPIInterface.dll, ...)
# with native backends in:
#     arm64_bin/libMotionSynergyAPINative.so
#     arm64_bin/libSmartStageAxis.so
#
# Those .so files are Linux ARM64 ELF binaries built for Ubuntu 22.04.
# They CANNOT be loaded on macOS (macOS needs Mach-O .dylib).
#
# IXXAT VCI driver is Windows-only.
# The IXXAT_SocketCAN tarball in Drivers/ is a Linux kernel driver.
# There is no macOS driver for the IXXAT USB-to-CAN V2 adapter.
#
# Docker Desktop on macOS runs Linux in a VM with NO USB passthrough,
# so a container on Mac cannot see the IXXAT adapter either.
#
# Conclusion: the host that physically owns the IXXAT cable must run either
# Ubuntu (preferred, matches the shipped libs) or Windows (Pro-Motion native).
# macOS cannot directly drive the stage.
#
# =============================================================================
# RECOMMENDED DEPLOYMENT TOPOLOGY
# -----------------------------------------------------------------------------
#
#                           LAN (TCP/HTTP/ZeroMQ)
#   +-----------------+    move_abs(mm)    +-------------------+
#   |  Mac (this box) | -----------------> |  Linux side host  |
#   |                 |                    |  (RPi 5 / NUC,    |
#   |  OCT pipeline   | <----------------- |   Ubuntu 22.04)   |
#   |  surface.py     |  position_mm,      |                   |
#   |  control.py     |  status            |  MotionSynergyAPI |
#   |  RemoteDriver   |                    |  (.NET on Linux)  |
#   +-----------------+                    +---------+---------+
#                                                    | USB
#                                                    v
#                                          +-------------------+
#                                          |  IXXAT USB-to-CAN |
#                                          +---------+---------+
#                                                    | CAN 2.0B
#                                                    v
#                                          +-------------------+
#                                          |   DOF-5 stage     |
#                                          +-------------------+
#
# - dof_driver.py on the Mac implements RemoteDriver (HTTP client).
# - A small server on the Linux box wraps MotionSynergyAPI and exposes:
#       POST /home
#       POST /move_abs   {mm: float}
#       GET  /position   -> {mm: float, homed: bool, error: str|null}
#       POST /stop
# - control.py never knows which driver it's talking to. It just calls
#   driver.move_relative(units) (or move_absolute) on the abstract interface.
#
# Until the Linux host exists, we develop against MockDriver and replay
# saved OCT frames (already supported by oct_source.py).
#
# =============================================================================
# DIAGNOSTIC MOVEMENTS (separate from control.py)
# -----------------------------------------------------------------------------
# Diagnostics belong in scripts/, not in this file. They use dof_driver
# directly, bypassing the control law:
#
#   scripts/dof_diagnostic.py  - interactive CLI (recommended first):
#       commands: home, pos, mv <mm>, abs <mm>, stop, sweep, quit
#       prints: current position, last command latency, error flags
#
# A web GUI is nicer for non-developers but slower to build and adds Flask/
# FastAPI + a frontend. Defer until the CLI proves the loop works.
#
# =============================================================================
# IMPLEMENTATION PLAN (we'll write this together, line by line)
# -----------------------------------------------------------------------------
#   1. ControlOutput dataclass
#        error_px, error_um, move_command, clipped, in_deadband, filtered_y
#   2. Controller class with state (filtered_y) and a step(center_y) method
#   3. Pixel -> um -> DOF-units conversion using config
#   4. EMA filter on center_y
#   5. Compute error vs target, apply deadband, apply clip
#   6. Return ControlOutput
#
# =============================================================================

