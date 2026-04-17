# UW_BIOENGINEERING_DoF-5
Please reach out to antbloch@uw.edu for access to DoF-5 specific software.
(Software is not publicly available)

# DoF-5 First-Time Initialization

> **Product:** DOF-5 / DOF-9 Objective Focuser  
> **Manufacturer:** Dover Motion (159 Swanson Rd, Boxborough, MA 01719)  
> **Reference Docs:** P/N 41-1114 Rev D (User Guide), P/N 41-1212 Rev A (Software Guide), P/N 41-1598 Rev C (Quick Start)  
> **Last Updated:** April 2026

---

## Table of Contents

- [Overview](#overview)
- [What Ships in the Box](#what-ships-in-the-box)
- [Cable & Adapter Kits](#cable--adapter-kits)
- [Hardware Preparation](#hardware-preparation)
  - [1. Remove the Shipping Stop](#1-remove-the-shipping-stop)
  - [2. Verify DIP Switch Settings](#2-verify-dip-switch-settings)
  - [3. Mount the Stage](#3-mount-the-stage)
- [Wiring](#wiring)
  - [Stage Connector Pinout (Gecko 16-Pin)](#stage-connector-pinout-gecko-16-pin)
  - [HD-15 Breakout Board Pinout](#hd-15-breakout-board-pinout)
  - [Power Wiring](#power-wiring)
  - [CAN Wiring](#can-wiring)
  - [RS-232 Wiring](#rs-232-wiring)
  - [RS-485 Wiring](#rs-485-wiring)
  - [Step & Direction Wiring](#step--direction-wiring)
  - [Full Wiring Diagram (CAN Example)](#full-wiring-diagram-can-example)
- [Software Installation](#software-installation)
- [First-Time Initialization (Pro-Motion)](#first-time-initialization-pro-motion)
  - [Step 1 — Connect](#step-1--connect)
  - [Step 2 — Set Units](#step-2--set-units)
  - [Step 3 — Drive Signal Scaling](#step-3--drive-signal-scaling)
  - [Step 4 — Analog Calibration](#step-4--analog-calibration)
  - [Step 5 — Enable the Servo](#step-5--enable-the-servo)
  - [Step 6 — Home the Stage](#step-6--home-the-stage)
  - [Step 7 — Test Motion](#step-7--test-motion)
- [Specifications Quick Reference](#specifications-quick-reference)
- [Troubleshooting](#troubleshooting)
- [Contact](#contact)

---

## Overview

The DOF (Dover Objective Focuser) is a single-axis linear stage with an integrated servo controller, encoder, and voice-coil motor. It is designed for precision objective focusing with nanometer-level resolution. Two travel variants exist:

| Model | Total Travel | Hard Stop Travel | Payload Range |
|-------|-------------|-----------------|---------------|
| DOF-5 | 5 mm        | 6 mm            | 0–900 g       |
| DOF-9 | 9 mm        | 9 mm            | 0–1500 g      |

Communication is supported via **RS-232**, **RS-485** (2-wire / 4-wire), **CAN 2.0B**, and **Step & Direction** (TTL).

---

## What Ships in the Box

| Item | Description |
|------|-------------|
| DOF-5 or DOF-9 stage | With objective bracket (if ordered) |
| USB stick drive | Contains Pro-Motion installer, SDK, config file, FIR report, docs |
| Shipping stop hardware | 2× M2.5x16 screws + 1 shoulder screw (pre-installed) |
| Interface cable kit | Ordered separately — see below |

### Files on the USB Stick

```
├── 41-1601/                    # MOXA driver installer (RS-232/RS-485 kits only)
├── MotionSynergyAPI_x.x.xxxxx # Software Development Kit
├── 41-1114.pdf                 # DOF User Guide
├── 41-1212.pdf                 # DOF-5 Pro-Motion Software Guide
├── 41-1598.pdf                 # DOF-5 Quick Start Guide (RS-232)
├── FIR_SN_######.pdf           # Final Inspection Report (your serial number)
└── ######_ConfigScript.txt     # Factory configuration file (your serial number)
```

---

## Cable & Adapter Kits

All kits include the **HD-15 Breakout Module Cable (36086-00)** — a braided Gecko-to-HD-15 cable plus a screw-terminal breakout board.

| Kit P/N | Contents | Adapter |
|---------|----------|---------|
| **36086-00** | HD-15 cable + breakout board only | — |
| **36100-00** | HD-15 cable + breakout + power cable + RS-232 cable | Moxa UPort 1150 (USB-to-Serial) |
| **36101-00** | HD-15 cable + breakout + power cable + RS-485 cable | Moxa UPort 1150 (USB-to-Serial) |
| **36102-00** | HD-15 cable + breakout + power cable + CAN cable | IXXAT USB-to-CAN V2 compact |

The kit's secondary cable has **loose labeled wires** on one end — these connect to the breakout board screw terminals and to the power supply / adapter.

---

## Hardware Preparation

### 1. Remove the Shipping Stop

> ⚠️ **WARNING:** Powering on the stage with the shipping stop installed will damage the stage.

Using a **2 mm hex wrench**, remove:
- 2× M2.5x16 socket head cap screws
- 1× shoulder screw

**Save these parts** — re-install them if you ever need to ship the stage.

### 2. Verify DIP Switch Settings

The DIP switches are on the **back** of the stage (visible through a small window). Set them **before** mounting.

```
Switch positions viewed from back:

    ┌──────────────────┐
    │  O Z             │
    │                  │
    │  ▪  ▪  ▪  ▪     │
    │  1  2  3  4      │
    └──────────────────┘
    O = OFF (down)   Z = ON (up)
```

| Protocol | SW1 | SW2 | SW3 | SW4 |
|----------|-----|-----|-----|-----|
| **RS-232** | ON | ON | any | any |
| **RS-485** Full Duplex, Low Z | ON | OFF | OFF | ON |
| **RS-485** Half Duplex, Low Z | ON | OFF | OFF | OFF |
| **RS-485** Full Duplex, High Z | ON | OFF | ON | ON |
| **RS-485** Half Duplex, High Z | ON | OFF | ON | OFF |
| **CAN 2.0B** Low Z (single unit) | OFF | any | OFF | any |
| **CAN 2.0B** High Z (multi-drop, not last) | OFF | any | ON | any |

**Notes:**
- **Low Z** = 120Ω termination. Use for single unit or last unit on a bus.
- **High Z** = 125kΩ termination. Use for all units except the last on a multi-drop bus.
- For CAN mode, switches 2 and 4 are ignored.

### 3. Mount the Stage

| Orientation | Condition |
|-------------|-----------|
| **Vertical** (counterbalance on top) | Stage ordered with counterbalance |
| **Horizontal** | Stage ordered without counterbalance |

**Requirements:**
- Mounting surface flatness: **≤ 25 µm** (0.001")
- Attach the **correct payload mass** (objective, lens, or dummy mass) **before powering on**. Without load the stage will be unstable and buzz.
- Torque: **1.0 Nm** for M3 (front mount), **2.5 Nm** for M4 (rear mount), stainless hardware.

---

## Wiring

### Stage Connector Pinout (Gecko 16-Pin)

The stage uses a **Harwin Gecko G125-MH11605L7P** (16-pin, 1.25mm pitch).  
Mating connector: **Harwin G125-2241696F1**.

| Pin | RS-232 | RS-485 | CAN | Notes |
|-----|--------|--------|-----|-------|
| 1 | — | — | — | Factory Use Only |
| 2 | — | — | — | Factory Use Only |
| 3 | NC | Z (RxD−) | NC | |
| 4 | TX | Y (RxD+) | NC | TX/RX from DOF's perspective |
| 5 | NC | B (TxD−) | CANH | |
| 6 | RX | A (TxD+) | CANL | |
| 7 | | **Supply Return (PGND)** | | Power ground |
| 8 | | **Motor Bus Supply (+VP)** | | 24VDC |
| 9 | — | — | — | Factory Use Only |
| 10 | — | — | — | Factory Use Only |
| 11 | | **Digital Input A (Step)** | | TTL 3.3–5V, max 4.8 MHz |
| 12 | | **Digital Input B (Direction)** | | TTL 3.3–5V |
| 13 | | **Digital Output** | | |
| 14 | | **Digital Return (DGND)** | | Signal / comm ground |
| 15 | | **Supply Return (PGND)** | | Power ground |
| 16 | | **Logic Supply (+VL)** | | 24VDC |

> **Both Pin 8 (+VP) and Pin 16 (+VL) must be powered** for the stage to operate. They are isolated on the board. Pins 7 and 15 are power ground returns. Pin 14 is digital/comm ground only (low current).

### HD-15 Breakout Board Pinout

When using the HD-15 Breakout Cable, the Gecko pins map to the HD-15 as follows:

| HD-15 Position | Signal | Gecko Pin |
|----------------|--------|-----------|
| 1 | Factory Use Only | 1 |
| 2 | Factory Use Only | 2 |
| 3 | NC / RxD− / NC | 3 |
| 4 | TX / RxD+ / NC | 4 |
| 5 | NC / TxD− / CANH | 5 |
| 6 | RX / TxD+ / CANL | 6 |
| 7 | PGND (Supply Return) | 7 |
| 8 | +VP (Motor Bus Supply) | 8 |
| 9 | Factory Use Only | 9 |
| 10 | Factory Use Only | 10 |
| 11 | INPUT 1 (Step) | 11 |
| 12 | INPUT 2 (Direction) | 12 |
| 13 | OUTPUT | 13 |
| 14 | DGND (Digital Return) | 14 |
| 15 | +VL (Logic Supply) | 16 |

> ⚠️ Note: HD-15 Position 15 = Gecko Pin **16** (Logic Supply). Gecko Pin 15 (second PGND) is mapped to HD-15 Position 7 (shared with Pin 7).

---

### Power Wiring

**Required:** 24VDC ±10% power supply, **≥ 1.5A** recommended (25W peak motor + 2W logic).

| Breakout Pos | Signal | Wire To |
|-------------|--------|---------|
| **8** | +VP (Motor Bus) | +24V terminal on PSU |
| **15** | +VL (Logic) | +24V terminal on PSU |
| **7** | PGND | GND terminal on PSU |

> ⚠️ Use **26 AWG or thicker** wire for power signals. Higher gauge wires risk overheating.

---

### CAN Wiring

**Breakout Board → IXXAT USB-to-CAN V2 (D-Sub 9)**

| Breakout Pos | Signal | IXXAT D-Sub 9 Pin |
|-------------|--------|-------------------|
| **5** | CANH | **7** |
| **6** | CANL | **2** |
| **14** | DGND | **3** |

**Defaults:** Baud = 1 Mbaud (factory), Node ID = 0.  
**Pro-Motion default:** Baud = 20k. Match accordingly.

---

### RS-232 Wiring

**Breakout Board → Moxa UPort 1150 (DB-9)**

| Breakout Pos | Signal | DB-9 Pin |
|-------------|--------|----------|
| **4** | TX (from DOF) | **2** (RX on adapter) |
| **6** | RX (from DOF) | **3** (TX on adapter) |
| **7** | PGND | **5** (GND) |

**Defaults:** Baud = 57600, 8N1, Point-to-Point.

---

### RS-485 Wiring

**Breakout Board → Moxa UPort 1150 (DB-9, 4-wire full duplex)**

| Breakout Pos | Signal | DB-9 Pin |
|-------------|--------|----------|
| **5** | TxD− (from DOF) | **1** |
| **4** | TxD+ (from DOF) | — |
| **6** | RxD+ (from DOF) | **3** |
| **3** | RxD− (from DOF) | **4** |
| **14** | DGND | **5** |

> For **2-wire half duplex**, TxD and RxD pairs are tied together. Use protocol mode "Multi-drop using idle line detection" in Pro-Motion.

**Defaults:** Baud = 57600, Address = 0.

---

### Step & Direction Wiring

| Breakout Pos | Signal | Source |
|-------------|--------|--------|
| **11** | Step (Pulse) | TTL output from motion controller |
| **12** | Direction | TTL output from motion controller |
| **14** | DGND | Ground reference |

**Specs:** TTL 3.3V–5V, max frequency **4.8 MHz**.

Requires software configuration in Pro-Motion to enable electronic gearing mode (see Software Guide p.19–21).

---

### Full Wiring Diagram (CAN Example)

```
                                    ┌──────────────────────────┐
                                    │      24V DC PSU          │
                                    │   +V ─────┬──────── GND  │
                                    └───────────┼──────────┼───┘
                                                │          │
                         ┌──────────────────────┼──────────┼──────┐
                         │  HD-15 BREAKOUT BOARD │          │      │
                         │                      │          │      │
                         │  Pos 8  (+VP) ◄──────┘          │      │
                         │  Pos 15 (+VL) ◄──────┘          │      │
                         │  Pos 7  (PGND) ◄────────────────┘      │
                         │                                        │
  ┌───────────┐          │  Pos 5  (CANH) ──────────┐             │
  │  DOF-5    │  Gecko   │  Pos 6  (CANL) ────────┐ │             │
  │  Stage    ├══════════╡  Pos 14 (DGND) ──────┐ │ │             │
  │           │  Cable   │                      │ │ │             │
  └───────────┘          └──────────────────────┼─┼─┼─────────────┘
                                                │ │ │
                                    ┌───────────┼─┼─┼────────────┐
                                    │  IXXAT    │ │ │             │
                                    │  D-Sub 9  │ │ │             │
                                    │           │ │ │             │
                                    │  Pin 3 ◄──┘ │ │  (DGND)    │
                                    │  Pin 2 ◄────┘ │  (CANL)    │
                                    │  Pin 7 ◄──────┘  (CANH)    │
                                    │                             │
                                    │         USB ────────────┐   │
                                    └─────────────────────────┼───┘
                                                              │
                                                    ┌─────────┴──┐
                                                    │   Laptop    │
                                                    │ (Pro-Motion)│
                                                    └────────────┘
```

---

## Software Installation

### 1. Install the adapter driver

| Protocol | Driver |
|----------|--------|
| RS-232 / RS-485 | Moxa UPort driver from USB stick (`41-1601/`) |
| CAN | IXXAT VCI driver from [HMS/IXXAT website](https://www.ixxat.com) |

### 2. Install Pro-Motion

From the USB stick, run **`Pro-Motion5.20.exe`**.

> ⚠️ The DOF requires **Pro-Motion version 5.20** specifically.

---

## First-Time Initialization (Pro-Motion)

### Step 1 — Connect

1. Power on the 24V supply → confirm **red LED** on stage
2. Connect the adapter (Moxa or IXXAT) to laptop via USB
3. Open Pro-Motion
4. At the Interface dialog:

| Protocol | Selection | Settings |
|----------|-----------|----------|
| RS-232 | COM | Baud: 57600, Parity: None, Stop bits: 1, Protocol: Point to point, Address: 0 |
| RS-485 | COM | Baud: 57600, Protocol: Point to point (4-wire) or Multi-drop (2-wire), Address: 0 |
| CAN | CAN | Baud: 20k, Node ID: 0 |

5. Click **OK** → Pro-Motion connects and populates the Axis Control panel

### Step 2 — Set Units

1. Click **Units** (bottom right of Axis Control panel)
2. Select **Linear**
3. Enter the encoder resolution:

| Encoder | Counts/mm |
|---------|-----------|
| 5 nm (E38) | 200,000 |
| 1.25 nm (E39) | 800,000 |

4. Set unit to **Millimeter**
5. Check your FIR report (`FIR_SN_######.pdf`) for which encoder you have

### Step 3 — Drive Signal Scaling

1. Still in the Units window, click **Drive Signal Scaling...**
2. Set:

| Field | Value |
|-------|-------|
| Leg currents | **0.1611** mA/count |
| Bus current | **0.1007** mA/count |
| Bus voltage | **0.9663** mV/count |

3. Click **Apply**, then **OK**
4. Click **OK** on the Units window

### Step 4 — Analog Calibration

> This must be done before enabling the servo for the first time after power-on.

1. Click **Motor Control** → set Motor Command to **0** → **Apply** → **OK**
2. Click **Operating Mode** → click **Disable All**
3. Check **Axis enable** and **Motor output** checkboxes → click **Apply**
4. Click **Current Loop** → click **Analog calibration...**
5. Click **Zero!** → **Apply**
6. Click **Calibrate!** → **Apply** → **OK**

### Step 5 — Enable the Servo

1. Click **Operating Mode**
2. Click **Enable All**
3. If prompted *"Current feedback signals are not calibrated. Calibrate now?"* → click **No** (already done in Step 4)
4. Verify **all five Active indicators** are filled (Trajectory, Position loop, Current loop, Motor output, Axis enable)
5. The stage should now hold position

### Step 6 — Home the Stage

> Required after every power-on or servo disable before position moves can be made.

1. Click **Homing**
2. Configure:

| Parameter | Value |
|-----------|-------|
| Homing method | **Home signal** |
| Direction | **Positive** |
| Velocity | 200,000 counts/sec (= 1 mm/s for 5 nm encoder) |
| Timeout | 30 seconds |
| Distance to move away from homing trigger | 0 counts |

3. Click **Start!**
4. Wait for **"Completed"** status
5. Position register is now zeroed at mid-travel

### Step 7 — Test Motion

1. Click **Trajectory**
2. Set Profile mode: **Trapezoidal**
3. Set parameters:

| Parameter | Example Value |
|-----------|--------------|
| Acceleration | 1000 mm/s² |
| Deceleration | 1000 mm/s² |
| Velocity | 1 mm/s |

4. Shuttle mode: **Single move**
5. Enter Position 1: **0.5** mm
6. Click **Go**

> ⚠️ All moves are **absolute**, not relative.

**Other modes:**
- **Manual** — alternates between Position 1 and Position 2 on each Go click
- **Automatic** — continuously cycles between two positions with a configurable dwell time

---

## Specifications Quick Reference

| Parameter | DOF-5 | DOF-9 | Units |
|-----------|-------|-------|-------|
| Total Travel | 5 | 9 | mm |
| Full Travel Accuracy (2σ) | 5 | 5 | µm |
| Bi-directional Repeatability (2σ) | 1 | 1 | µm |
| Bi-dir Repeatability, 500nm move (2σ) | ≤ 25 | ≤ 25 | nm |
| Home Repeatability (2σ) | ≤ 1 | ≤ 1 | µm |
| Minimum Move | 15 | 15 | nm |
| Servo Bandwidth | > 225 | > 225 | Hz |
| Move & Settle (100nm, ±15nm) | ≤ 15 | ≤ 15 | ms |
| Move & Settle (250nm, ±15nm) | ≤ 15 | ≤ 15 | ms |
| Servo Stability | ≤ 5 | ≤ 5 | nm rms |
| Max Velocity (1.25nm encoder) | 30 | 30 | mm/s |
| Max Velocity (5nm encoder) | 125 | 125 | mm/s |
| Max Acceleration (1kg payload) | 6 | 6 | m/s² |
| Weight (with std. objective mount) | 0.5 | 0.6 | kg |
| Power (recommended) | 24VDC ±10% | 24VDC ±10% | |
| Current (max / typical) | 1.125 / 0.200 | 1.125 / 0.200 | A |
| Operating Temperature | 5–40 | 5–40 | °C |
| Relative Humidity | 20–80% non-condensing | 20–80% non-condensing | |

---

## Troubleshooting

### Can't connect to the stage
- Confirm **red LED** is lit (24V power applied)
- Verify correct COM port number in Device Manager (RS-232/RS-485)
- Verify DIP switch settings match your protocol
- For CAN: ensure IXXAT driver is installed and Node ID / baud rate match

### Stage is unstable or buzzing
- Ensure correct **payload mass** is attached
- Verify correct **mounting orientation** (counterbalance on top for vertical)
- Ensure mounting surface is flat (≤ 25 µm)
- Re-load factory config: Device → NVRAM → browse to `######_ConfigScript.txt` → Download!
- Contact Dover Motion if re-tuning is needed

### "Current feedback signals are not calibrated" error
- If you accidentally clicked **Yes**: perform a power cycle (preferred) or follow the manual analog calibration recovery procedure (Software Guide p.24–25)
- Always click **No** if you already completed analog calibration

### Stage won't move after enabling
- Verify **all five** operating mode indicators are active
- Ensure you have **homed** the stage after power-on
- Check that trajectory values are within travel limits

### Adjustable Hard Stops
- DOF-5: ±3 mm (adjustable down to ±1.7 mm per side)
- DOF-9: ±4.5 mm (adjustable down to ±3.2 mm per side)
- Use a **5 mm hex key**; clockwise rotation reduces travel

---

## Contact

**Dover Motion**  
159 Swanson Rd, Boxborough, MA 01719 USA  
Phone: (508) 475-3400  
Email: service@dovermotion.com  
Web: [www.dovermotion.com](http://www.dovermotion.com)

---

## License

This document is a user-created summary derived from Dover Motion's publicly provided product documentation. All product specifications, trademarks, and connector references belong to their respective owners (Dover Motion / Dover Corporation, Harwin plc, Moxa Inc., HMS/IXXAT).
