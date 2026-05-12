# Coil Winder Controller

This project implements a custom controller for a DIY coil winding machine using a **Makerbase MKS DLC32 V2.1** board (an ESP32-based CNC motherboard). 

## Hardware Setup

The mechanical base of the machine is built from an i3-style 3D printer, modified for coil winding:
- **Traverse Axis:** Built using the former Z-axis of the 3D printer. It uses two parallel stepper motors (one on each side). These are connected to the **X** and **Y** stepper motor outputs on the DLC32 board.
- **Spindle Axis:** Driven by a strong, dedicated stepper motor connected to the **Z** stepper motor output on the DLC32.
- **Traverse Endstops:** Two SparkFun limit switches are used for homing and tramming the traverse axis. They are wired to the **X-limit (Pin 36)** and **Y-limit (Pin 35)** ports. They use a standard active-LOW trigger state.
- **Emergency Stop / Probe:** A limit switch connected to the **Probe (Pin 22)** port acts as a hardware emergency stop to immediately halt all motion when triggered.

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

### Features:

1. **Custom Stepper Driver Implementation:** 
   - Overrode the `AccelStepper` library with custom `ShiftStepper` and `TraverseStepper` classes to route Step/Dir signals through the I2S shift register.
2. **Synchronized Traverse Motion & Tramming:**
   - Both the X and Y stepper outputs are controlled simultaneously to move the dual-motor traverse axis smoothly.
   - During the homing sequence, each motor moves independently when it reaches its respective limit switch, allowing the machine to perfectly tram (square) itself.
3. **Web Interface (WebUI):**
   - The ESP32 hosts a responsive local web server (accessible via `http://coilwinder.local` using mDNS, or via the fallback SoftAP `CoilWinder`).
   - Real-time stats monitor position, homing state, speed, and hardware limit switch status.
   - Users can manually configure Start/End positions and Target turns.
4. **Jog Controls:**
   - The UI provides buttons to jog the traverse axis in +/- 1mm and 10mm increments.
   - Built-in buttons allow setting the configured start or end traverse positions to the current physical position.
5. **Safety Features:**
   - Software limit switch checks during jogging and running to prevent physical crashing.
   - Hardware E-STOP button mapped to the Probe pin stops all motion instantly.
6. **Live Speed Overrides:**
   - Quick UI toggles allow scaling speeds from 50% to 200% mid-winding.

## Future Plans
- Integrate a 2.0" TFT display and a rotary encoder to provide an onboard UI, allowing operation without needing the WebUI.
