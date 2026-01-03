# Wireless PID Motor Control System

![Arduino](https://img.shields.io/badge/Arduino-Uno-00979D?style=flat-square&logo=arduino&logoColor=white)
![LabVIEW](https://img.shields.io/badge/LabVIEW-FFDB00?style=flat-square&logo=labview&logoColor=black)
![Control](https://img.shields.io/badge/Control-PID-blue?style=flat-square)
![Wireless](https://img.shields.io/badge/Wireless-nRF24L01-green?style=flat-square)

A wireless motor control system implementing closed-loop PID speed regulation with multi-sensor feedback. Features real-time data acquisition, LabVIEW integration, and bidirectional RF communication.

---

## Overview

This project implements a complete closed-loop motor control system with wireless command transmission and real-time sensor monitoring. The system uses a PID controller to maintain target motor speed based on light sensor input, with environmental monitoring and data logging capabilities.

The project demonstrates practical application of control systems theory, embedded programming, wireless communication, and hardware integration for industrial automation applications.

**Course:** ECE 3709L - Control Systems Laboratory  
**Institution:** Cal Poly Pomona  
**Date:** Fall 2024

---

## System Architecture

### Transmitter Module
- Arduino Uno with nRF24L01 wireless transceiver
- LCD display for command/status visualization
- Serial command interface
- Receives status updates from receiver

### Receiver Module (Motor Controller)
- Arduino Uno with multiple sensors and actuators
- DC motor with L293D H-bridge driver
- PID speed controller with anti-windup
- Multi-sensor integration (DHT11, LDR, LM393 speed sensor module)
- Real-time feedback display

### Data Acquisition
- LabVIEW interface for monitoring and logging
- Real-time graphing of RPM, PWM, and sensor data
- Parameter tuning and system analysis

---

## Features

**Closed-Loop Control**
- PID speed regulation with tunable parameters (Kp, Ki, Kd)
- Anti-windup protection for integral term
- Adaptive target speed based on LDR input
- Manual/PID mode switching

**Wireless Communication**
- Bidirectional RF link using nRF24L01 (2.4 GHz)
- Command transmission (Forward/Reverse/Stop)
- Status feedback with PID mode indication
- Packet-based protocol with error handling

**Multi-Sensor Integration**
- DHT11: Temperature and humidity monitoring
- LDR: Light-dependent speed control input
- Rotary encoder: Real-time RPM measurement via interrupts
- I2C LCD: Local status display

**Motor Control**
- L293D H-bridge for bidirectional control
- PWM speed regulation (85%-100% duty cycle)
- Encoder-based speed feedback (40-slot disk)
- Emergency stop capability

**User Interface**
- Physical buttons (Start, Stop, PID Enable)
- Dual LCD displays (transmitter and receiver)
- Serial command interface
- LabVIEW data visualization

---

## Hardware Components

**Microcontroller**
- 2× Arduino Uno (ATmega328P)
- 16 MHz clock, 32 KB Flash, 2 KB SRAM

**Wireless Communication**
- 2× nRF24L01+ 2.4 GHz RF transceivers
- SPI interface, 250 kbps - 2 Mbps data rate

**Motor & Driver**
- DC motor (12V, adjustable speed)
- L293D dual H-bridge motor driver
- PWM-controlled enable pin (Pin 11)

**Sensors**
- DHT11: Digital temperature & humidity sensor
- LDR (Light Dependent Resistor) with voltage divider
- LM393 rotary encoder module (40 slots)

**Display & Interface**
- 2× 16×2 I2C LCD displays (HD44780 compatible)
- 3× Pushbutton switches (Start, Stop, PID Enable)
- Debouncing implemented in software

**Power Supply**
- 5V for Arduino and logic circuits
- 12V for DC motor (separate supply recommended)
- 3.3V for nRF24L01 via AMS1117-based linear regulator module

---

## Control System Design

### PID Controller

The system implements a discrete-time PID controller to regulate motor speed:

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

Where:
- `u(t)` = PWM correction (added to base PWM)
- `e(t)` = error (target RPM - measured RPM)
- `Kp` = Proportional gain (2.0)
- `Ki` = Integral gain (0.5)
- `Kd` = Derivative gain (0.1)

**Implementation Features:**
- Update rate: 20 Hz (50 ms interval)
- Anti-windup: Integral clamped to ±100
- Derivative kick prevention
- Error deadband for steady-state stability

### Speed Control

**Manual Mode:**
- LDR directly mapped to PWM (85%-100%)
- Immediate response, no feedback regulation

**PID Mode:**
- Target RPM = f(LDR value), range: 50-200 RPM
- Base PWM: 217 (85% duty cycle)
- PID correction: ±38 (enables 85%-100% range)
- Closed-loop regulation maintains setpoint

### RPM Measurement

**Encoder Configuration:**
- 40-slot optical encoder disk
- Interrupt-driven pulse counting (Pin 2)
- RPM calculation every 100 ms
- Formula: `RPM = (pulses × 60000) / (slots × time_ms)`

---

## Software Architecture

### Main Control Loop (Receiver)

```
Loop iteration (non-blocking):
├── Calculate RPM from encoder pulses
├── Update motor PWM (PID or manual)
├── Debounce physical buttons
│   ├── Start button → Forward
│   ├── Stop button → Stop
│   └── PID button → Toggle mode
├── Read sensors (every 2s)
│   ├── DHT11 temperature/humidity
│   ├── LDR value
│   └── RPM calculation
├── Update LCD display (every 1s)
├── Check for RF commands
└── Send status update if state changed
```

**Timing Strategy:**
- Critical tasks (RPM, PWM) execute every loop (~10 ms)
- Sensor reads every 2 seconds (DHT11 slow)
- Display updates every 1 second (I2C slow)
- Non-blocking execution for real-time performance

### Communication Protocol

**Transmit (TX → RX):**
- Command string: `"Forward"`, `"Reverse"`, `"Stop"`
- 32-byte packet, null-terminated

**Receive (RX → TX):**
- Status string: `"State|PID_MODE"`
- Example: `"Forward|PID_ON"` or `"Stop|PID_OFF"`
- Sent on state change or PID mode toggle

---

## Pin Configuration

### Receiver (Motor Controller)

| Pin | Function | Component |
|-----|----------|-----------|
| D2 | Encoder Input | LM393 encoder (Interrupt) |
| D3 | Motor IN1 | L293D pin 2 (PWM capable) |
| D4 | Motor IN2 | L293D pin 7 |
| D5 | Start Button | Pushbutton (INPUT_PULLUP) |
| D6 | DHT11 Data | Temperature/humidity sensor |
| D7 | RF CE | nRF24L01 chip enable |
| D8 | RF CSN | nRF24L01 chip select |
| D9 | Stop Button | Pushbutton (INPUT_PULLUP) |
| D10 | PID Enable | Pushbutton (INPUT_PULLUP) |
| D11 | Motor Enable | L293D enable (PWM output) |
| A4 | I2C SDA | LCD display |
| A5 | I2C SCL / LDR | LCD display / Light sensor |
| SPI | MOSI/MISO/SCK | nRF24L01 communication |

### Transmitter

| Pin | Function | Component |
|-----|----------|-----------|
| D7 | RF CE | nRF24L01 chip enable |
| D8 | RF CSN | nRF24L01 chip select |
| A4 | I2C SDA | LCD display |
| A5 | I2C SCL | LCD display |
| SPI | MOSI/MISO/SCK | nRF24L01 communication |

---

## LabVIEW Integration

**Data Acquisition:**
- Serial communication with receiver Arduino
- Real-time plotting of RPM, PWM, and LDR values
- PID tuning parameter adjustment
- Data logging for post-processing

**Virtual Instrumentation:**
- Live RPM gauge
- PWM duty cycle indicator
- Temperature and humidity display
- Mode indicator (Manual/PID)

**Analysis Capabilities:**
- Step response characterization
- Settling time measurement
- Overshoot/undershoot analysis
- Steady-state error calculation

---

## PID Tuning Process

**Initial Parameters:**
- Started with Ziegler-Nichols method
- Ku (ultimate gain) ≈ 3.5
- Tu (oscillation period) ≈ 0.4s

**Final Tuned Values:**
- Kp = 2.0 (reduced from initial 2.1 to reduce overshoot)
- Ki = 0.5 (provides steady-state error elimination)
- Kd = 0.1 (dampens oscillations without noise amplification)

**Performance Metrics:**
- Rise time: ~0.8 seconds
- Settling time: ~2.5 seconds
- Overshoot: <10%
- Steady-state error: <2 RPM

---

## System Performance

**Speed Regulation:**
- Target range: 50-200 RPM
- Regulation accuracy: ±2 RPM in PID mode
- Response time: <3 seconds to setpoint
- PWM range: 85%-100% (217-255)

**Wireless Communication:**
- Range: ~30 meters (line of sight)
- Latency: <50 ms
- Packet success rate: >95%

**Sensor Accuracy:**
- RPM measurement: ±1 RPM
- Temperature: ±2°C (DHT11 spec)
- Humidity: ±5% RH (DHT11 spec)

---

## Code Structure

```
wireless-motor-control-pid/
├── Receiver/
│   └── receiver_pid_motor.ino     # Main motor controller
├── Transmitter/
│   └── transmitter_remote.ino     # Wireless command sender
├── LabVIEW/
│   └── motor_control_dashboard.vi # Data acquisition interface
└── README.md
```

---

## Installation & Usage

### Hardware Setup

1. **Wire transmitter Arduino:**
   - Connect nRF24L01 to SPI pins + CE(D7), CSN(D8)
   - Connect I2C LCD to A4(SDA), A5(SCL)
   - Power with USB

2. **Wire receiver Arduino:**
   - Connect nRF24L01 (same as transmitter)
   - Connect L293D motor driver with PWM on D11
   - Wire encoder to D2 (interrupt pin)
   - Connect DHT11 to D6
   - Wire LDR voltage divider to A5
   - Connect pushbuttons to D5, D9, D10 (with pullups)
   - Connect I2C LCD
   - Use separate 12V supply for motor

3. **Safety check:**
   - Verify motor power supply polarity
   - Confirm all grounds connected
   - Test buttons before motor operation

### Software Upload

1. Install Arduino IDE and required libraries:
   ```
   - RF24 (by TMRh20)
   - LiquidCrystal_I2C
   - DHT sensor library
   ```

2. Upload `receiver_pid_motor.ino` to receiver Arduino

3. Upload `transmitter_remote.ino` to transmitter Arduino

4. Open Serial Monitor on transmitter (9600 baud)

### Operation

**Serial Commands (Transmitter):**
- `F` or `f` → Forward
- `R` or `r` → Reverse  
- `S` or `s` → Stop

**Physical Buttons (Receiver):**
- Start button (D5) → Forward
- Stop button (D9) → Stop
- PID button (D10) → Toggle Manual/PID mode

**LabVIEW:**
- Open `motor_control_dashboard.vi`
- Select correct COM port
- Run VI to start data acquisition

---

## Testing & Validation

**Open-Loop Testing:**
1. Disable PID mode
2. Vary LDR coverage while motor running
3. Observe PWM change without speed regulation
4. Verify manual control response

**Closed-Loop Testing:**
1. Enable PID mode
2. Set target RPM by adjusting LDR
3. Introduce load on motor shaft
4. Verify PID compensates to maintain speed

**Communication Testing:**
1. Send commands via serial and buttons
2. Verify status updates on transmitter LCD
3. Test maximum range
4. Confirm bidirectional data flow

**Tuning Validation:**
1. Log step response data in LabVIEW
2. Calculate performance metrics
3. Adjust Kp, Ki, Kd for optimal response
4. Iterate until specifications met

---

## Learning Outcomes

**Control Systems:**
- PID controller design and implementation
- Closed-loop system analysis
- Stability and performance trade-offs
- Real-time control system constraints

**Embedded Systems:**
- Interrupt-driven programming
- Non-blocking code architecture
- Real-time task scheduling
- Multi-sensor integration

**Communication Systems:**
- SPI and I2C protocols
- Wireless RF communication
- Protocol design and debugging
- Error handling and reliability

**System Integration:**
- Hardware/software co-design
- Sensor calibration and validation
- Power supply management
- Signal conditioning

---

## Known Issues & Solutions

**Issue: I2C LCD updates cause timing jitter**  
Solution: Reduced LCD update rate to 1 Hz, cached display strings

**Issue: Encoder noise causing false counts**  
Solution: Added hardware pull-up resistor, interrupt on FALLING edge only

**Issue: PID integral windup during setpoint changes**  
Solution: Implemented integral clamping (±100 limit)

**Issue: Wireless packet loss at range**  
Solution: Reduced PA level to LOW, added status retransmission on change

---

## Future Enhancements

**Control Improvements:**
- Adaptive PID gains based on load
- Feedforward compensation
- Kalman filtering for encoder noise reduction

**Features:**
- Multiple motor control (expand to 2-4 motors)
- Speed profile following (ramp, trapezoidal)
- Data logging to SD card
- Web interface for remote monitoring

**Hardware:**
- Migrate to ESP32 for WiFi capability
- Add current sensing for torque estimation
- Implement regenerative braking

---

## References

**Control Theory:**
- Modern Control Engineering (Ogata)
- PID Controllers: Theory, Design, and Tuning (Åström & Hägglund)

**Datasheets:**
- ATmega328P Datasheet (Arduino Uno)
- nRF24L01+ Datasheet (Nordic Semiconductor)
- L293D Motor Driver Datasheet
- DHT11 Humidity & Temperature Sensor

**Application Notes:**
- Arduino Interrupt Handling
- I2C Communication Timing
- PID Tuning Guidelines (National Instruments)

---

## Acknowledgments

**Course:** ECE 3709L - Control Systems Laboratory  
**Instructor:** Sasoun Torousian  
**Institution:** California Polytechnic State University, Pomona

Special thanks to the Arduino and LabVIEW communities for libraries and documentation.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Contact

**Daniel Romero**  
Email: daniel.romero@ieee.org  
Portfolio: [electricalromero.com](https://electricalromero.com)  
LinkedIn: [linkedin.com/in/daniel-romero-ee](https://www.linkedin.com/in/daniel-romero-ee/)

---

*A practical implementation of closed-loop PID motor control with wireless communication, demonstrating control systems theory in embedded hardware.*

*Last Updated: December 2024*
