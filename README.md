# MiniSumo Robot Project

## Overview

This project showcases a fully autonomous MiniSumo robot designed and programmed for university-level robotics competitions. The robot's primary goal is to detect and push an opponent robot outside a circular ring, emulating the rules of traditional Japanese sumo wrestling.

## What is a MiniSumo Robot?

A MiniSumo robot is a small, self-operating robot that competes in an arena by attempting to push its opponent out of a circular ring (dojo). Robots must follow strict size (10x10 cm) and weight (max. 500g) limitations, and they must operate entirely without remote control.

## How It Works

The robot uses a mix of sensors and simple embedded programming to detect its environment and make decisions.

### Hardware Components

* **Microcontroller**: STM32F401RE (Nucleo board)
* **Motors**: Two DC gear motors controlled via H-bridge
* **Distance Sensor**: HC-SR04 Ultrasonic sensor
* **Line Sensors**: Two IR sensors to detect ring borders
* **Power Supply**: 12V LiPo battery regulated by an LM2596

###  Detection Logic

* **Opponent Detection**: The ultrasonic sensor detects opponents in front of the robot up to \~40 cm.
* **Edge Detection**: Infrared sensors detect the white border of the ring to avoid self-elimination.

## Software Implementation

The code is written in C using STM32CubeIDE with HAL (Hardware Abstraction Layer) libraries.

### Key Behaviors:

1. **Startup Sequence**:

   * Waits for the onboard button to be pressed.
   * Waits 5 seconds to comply with competition rules.

2. **Opponent Tracking**:

   * If an opponent is detected within range, the robot charges forward at full speed.

3. **Search Mode**:

   * If no opponent is detected, it spins in place to search.

4. **Safety Reactions**:

   * If a line sensor detects the white border, the robot quickly reverses and realigns.

### Peripherals Used

* **TIM3**: Input capture for ultrasonic echo timing
* **TIM4**: PWM generation for motor speed control
* **USART2**: UART for debugging messages

## 💬 Example UART Output

The robot sends messages via USB for real-time feedback:

```bash
Buscando...
Objetivo detectado
```

## Deployment

1. Flash the code to the STM32F401RE using STM32CubeIDE.
2. Power the system with a regulated 9–12V input.
3. Place the robot in the ring and press the blue button.

## Results

* **Success rate**: The robot was able to detect and push static opponents with high reliability.
* **Avoidance**: Efficient border detection and reversal reduced the chance of self-exit.

## Acknowledgements

Thanks to professors, teammates, and the embedded systems community for support and debugging guidance!

---

