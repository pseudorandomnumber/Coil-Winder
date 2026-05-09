# Coil Winder Controller

This project implements a custom controller for a DIY coil winding machine using a **Makerbase MKS DLC32 V2.1** board (an ESP32-based CNC motherboard). 

## Hardware Setup

The mechanical base of the machine is built from an i3-style 3D printer, modified for coil winding:
- **Traverse Axis:** Built using the former Z-axis of the 3D printer. It uses two parallel stepper motors (one on each side). These are connected to the **X** and **Y** stepper motor outputs on the DLC32 board.
- **Spindle Axis:** Driven by a strong, dedicated stepper motor connected to the **Z** stepper motor output on the DLC32.
- **Traverse Endstop:** A limit switch is used for homing the traverse axis. It is wired to the **Y-limit switch port** (GPIO 35) as a normally closed (NC) switch connected to ground.

### MKS DLC32 V2.1 Specifics

Unlike many standard ESP32 CNC boards, the MKS DLC32 V2.1 does not connect the stepper driver Step/Dir/Enable pins directly to the ESP32's GPIOs. Instead, it uses an **I2S Shift Register (74HC595)** to expand the IO for the stepper drivers.

- **I2S Pins:**
  - `BCK`: GPIO 16
  - `DATA`: GPIO 21
  - `WS` (Latch): GPIO 17
- **Shift Register Mapping:**
  - **X-Axis (Traverse Left):** Step = Bit 1, Dir = Bit 2
  - **Y-Axis (Traverse Right):** Step = Bit 5, Dir = Bit 6
  - **Z-Axis (Spindle):** Step = Bit 3, Dir = Bit 4
  - **Enable (Shared):** Bit 0 (Active Low)

## Software & Features

The firmware is developed using **PlatformIO** and the Arduino framework.

### What We Did So Far:

1. **Custom Stepper Driver Implementation:** 
   - Overrode the `AccelStepper` library with a custom `ShiftStepper` class to route Step/Dir signals through the I2S shift register instead of standard GPIO pins.
2. **Synchronized Traverse Motion:**
   - Both the X and Y stepper outputs are controlled simultaneously to move the dual-motor traverse axis smoothly.
3. **Web Interface (WebUI):**
   - The ESP32 hosts a local web server (accessible via `http://coilwinder.local` using mDNS, or via the fallback SoftAP `CoilWinder` / `coil1234`).
   - The UI allows users to configure:
     - Target number of turns
     - Traverse start and end positions (in mm)
     - Steps per mm
     - Traverse speed and Spindle speed
   - Controls to **Home Traverse**, **Start**, and **Stop** the winding process.
4. **Homing Sequence:**
   - Moves the traverse axis backwards until the normally closed (NC) limit switch on the Y-limit port triggers.
5. **Serial Command Interface:**
   - Support for basic text commands via Serial (e.g., `TURNS`, `STARTPOS`, `ENDPOS`, `SPEED`, `HOME`, `START`, `STOP`) for debugging and alternative control methods.

## Future Plans
- Integrate a 2.0" TFT display and a rotary encoder to provide an onboard UI, allowing operation without needing the WebUI.
