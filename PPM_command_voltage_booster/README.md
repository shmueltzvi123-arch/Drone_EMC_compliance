# Ground Station Signal Conditioning Suite (PPM Passthrough)

**Project:** Azrieli College of Engineering - EMC/EMI Drone Project

## 📌 Overview
This Arduino sketch provides a lightweight, ultra-low latency hardware interface between an RC transmitter (e.g., RadioMaster TX12) and an industrial optical transmitter (Broadcom HFBR-1521Z). 

## 💡 No Remote? (Alternative Code)
**If you do not have an RC transmitter that outputs a PPM signal, you do not need this code.** 
Instead, please navigate to our alternative codebase which configures the Arduino itself to act as a standalone remote control using 9 simple push buttons.

## 🎯 The Problem it Solves (Level Shifting)
This code is specifically designed for situations where your remote control successfully outputs the correct **PPM (Pulse Position Modulation) signal**, but at a **lower logic voltage (e.g., 3.3V)**. 

The Broadcom HFBR-1521Z Optical Transmitter requires a stable **5V TTL** square wave to drive its internal LED efficiently. Feeding it a 3.3V signal directly from the remote can result in weak optical transmission, signal drops, or complete failure to transmit the data through the plastic optical fiber (POF).

## 🛠️ The Solution
Instead of writing complex signal generation code from scratch, this script uses an **Arduino Uno** as an active hardware buffer and logic level shifter. 
1. It listens to the weak (3.3V) incoming PPM signal using hardware interrupts to guarantee zero jitter.
2. The moment a pulse is detected, the microcontroller instantly outputs a clean, stabilized **5V TTL pulse** (fixed at 300µs) to the optical transmitter.

## 🔌 Hardware Wiring Guide
* **PPM Signal from Remote** ➔ Arduino Pin `D2` *(Hardware Interrupt 0)*
* **GND from Remote** ➔ Arduino `GND`
* **Arduino Pin `D3`** ➔ Optical Transmitter `Data In`
* **Arduino `5V` & `GND`** ➔ Optical Transmitter `VCC` & `GND`

## ⚙️ How it Works under the Hood
The script utilizes an Interrupt Service Routine (ISR) triggered on the `RISING` edge of the incoming signal. Because it uses hardware interrupts instead of the standard `loop()`, it ensures minimum latency and prevents any clock jitter from affecting the critical timing parameters of the PPM frame delivery.