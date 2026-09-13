# Line Following Robot

An Arduino Uno R3-based robot that autonomously tracks a black line on a light-colored surface. Line position is detected using two independent IR sensor modules, and steering is achieved through differential control of a four-motor drivetrain via a single L298N-based motor driver.

**Overview**

The robot operates without manual input once powered on. It reads two IR sensors mounted at the front, determines the line's position relative to the chassis, and adjusts left/right motor speed accordingly to stay on course. If the line is not detected by either sensor, the robot executes a time-limited recovery maneuver before halting.

**Hardware Summary**

- Controller: Arduino Uno R3
- Line Detection: Two IR sensor modules (digital comparator output), each with an adjustable sensitivity threshold and an onboard status LED, positioned at the front of the chassis
- Drive System: Four DC gear motors, grouped into a left pair and a right pair
- Motor Control: One L298N-based dual-channel driver board (ENA, IN1, IN2, IN3, IN4, ENB)
- Power Supply: 3-cell 18650 Li-ion pack (approximately 11.1V nominal), routed through an inline power switch to the driver's motor supply input
- All components share a common ground reference

**Wiring Reference**

| Function | Arduino Pin |
|---|---|
| Left IR sensor output | A0 |
| Right IR sensor output | A1 |
| Left motor pair — speed (PWM) | 9 |
| Left motor pair — direction 1 | 8 |
| Left motor pair — direction 2 | 7 |
| Right motor pair — direction 1 | 5 |
| Right motor pair — direction 2 | 4 |
| Right motor pair — speed (PWM) | 10 |

**Control Logic**

With two sensors, the system cannot measure how far the line has drifted — only which side, if either, currently detects it. Control is therefore handled with a fixed-response lookup rather than a proportional controller:

| Left sensor | Right sensor | Interpretation | Response |
|---|---|---|---|
| Detects | Detects | Line centered, or a wide intersection | Move straight |
| Detects | No detection | Line has shifted left | Correct left |
| No detection | Detects | Line has shifted right | Correct right |
| No detection | No detection | Line not visible | Timed recovery, then stop |

During recovery, the robot pivots toward the side the line was last detected on. If the line is not reacquired within the configured timeout, the robot stops rather than continuing to search.

**Build Requirements**

No external libraries are needed; the sketch relies only on the standard Arduino core.

**Tunable Parameters**

```cpp
const bool LINE_IS_LOW = true;           // Output polarity when line is detected
const int BASE_SPEED = 130;              // Default cruising PWM value (0-255)
const int MAX_SPEED = 200;               // Upper PWM limit per motor
const int MIN_SPEED = 60;                // Lower PWM limit to prevent stalling
const int STEER_DIFFERENTIAL = 60;       // Correction amount applied during a turn
const unsigned long LOST_LINE_SEARCH_TIMEOUT_MS = 700; // Max recovery duration
```

All behavior tuning is handled through these constants at the top of the sketch.

**Known Issues and Fixes**

- No turning response at all: check `LINE_IS_LOW` — the sensor polarity may not match the code's assumption.
- Turns away from the line instead of toward it: same as above, reverse the `LINE_IS_LOW` value.
- Zig-zagging on straight sections: reduce `STEER_DIFFERENTIAL`.
- Weak response on sharp turns: increase `STEER_DIFFERENTIAL`, or increase the physical spacing between the two sensors.
- No motor movement: confirm the power switch is on and the driver is receiving voltage from the battery pack.

<img width="828" height="948" alt="image" src="https://github.com/user-attachments/assets/98a0dcf7-8f15-4380-89f2-094b5d4c75cf" />



