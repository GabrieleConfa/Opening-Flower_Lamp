# Mechatronic Floral Lamp

A personal mechatronics project consisting of a 5-DOF lamp with synchronized servo actuation and dynamically eased PWM lighting. The system utilizes kinematic control and a non-blocking software architecture to manage hardware concurrency.

## System Architecture and Control Strategy

To achieve synchronized movements and lighting, the system avoids standard linear delay functions. It relies on a non-blocking state machine and time-step integrated kinematics, allowing the microprocessor to handle multiple actuators concurrently.

### Finite State Machine
The core control loop operates on a finite state machine with four primary states: CLOSED, OPENING, OPEN, and CLOSING. State transitions are triggered by an external hardware interrupt and managed asynchronously. This architecture guarantees that reading sensor inputs, updating servo positions, and rendering LED frames occur concurrently without halting the microprocessor.

### Kinematic Servo Control
Rather than commanding the actuators to abruptly snap to static target angles, the software generates smooth trajectories dynamically:
* Time-Step Integration: The control loop calculates updates based on a strict delta-time interval of 25 ms.
* Velocity and Acceleration Profiles: Movement is governed by defined maximum velocities and an acceleration threshold. The system continuously calculates the required stopping distance and applies acceleration or deceleration to ensure smooth mechanical deployment.
* Phase Offsets: To simulate organic movement, the actuators are initialized with staggered phase delays and slight randomized positional jitters.

### Lighting Dynamics
The 12 WS2812B NeoPixels are synchronized directly with the kinematic progress of the physical deployment.
* Cubic Easing: A custom cubic ease-in-out function provides a natural, non-linear fade effect.
* Color Interpolation: RGB values are linearly interpolated from the base state to the active state during the opening sequence.
* Resource Management: NeoPixel data transmission temporarily disables system interrupts. To prevent standard hardware timer disruptions and eliminate servo PWM jitter, LED updates are strictly clamped to a minimum interval of 40 ms.

## Hardware Specifications

* Microcontroller: Arduino Uno / Nano (or compatible architecture)
* Actuators: 5x SG90 Micro Servos
* Lighting: 12x WS2812B NeoPixel LEDs
* Input: 1x Pushbutton (Active Low, utilizing internal pull-up resistor)
* Power: 5V External Power Supply (Required to support the current draw of the actuators and LEDs)

### Pin Configuration

* Input Button: D2
* Servos 1-5: D3, D4, D5, D6, D7
* NeoPixel Data: D8

## Repository Structure

* /src - Contains the main.ino C++ control software.
* /cad - Contains the Fusion 360 (CADflower.f3d) workspace for the 3D-printed mechanical components.
* /Pic - Cpntains images of the Lamp in different states.
