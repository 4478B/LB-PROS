# 4478B — VEX Push Back Robot (PROS)

PROS remake of our VEX 25-26 Push Back competition robot.

Built on **PROS** and **LemLib**.

## Frameworks

| Library | Role |
|---------|------|
| [PROS](https://pros.cs.purdue.edu/) | Real-time operating system for VEX V5. Provides tasks, motor/sensor APIs, and competition lifecycle hooks. |
| [LemLib](https://lemlib.readthedocs.io/) | Odometry, motion planning (moveToPoint / moveToPose), and PID chassis control. |

---

## Project Structure

```
src/
  main.cpp             # PROS entry point — lifecycle hooks + driver-control handlers
  devices.cpp          # Hardware object definitions (motors, sensors, pneumatics, chassis)
  auton_routes.cpp     # Every autonomous routine + motion helpers
  auton_selector.cpp   # Brain-screen autonomous selector
  extended_chassis.cpp # Custom chassis subclass with mid-motion clamping
  old_systems.cpp      # Motor-encoder PID straight drive (drivePID)
  testing.cpp          # Developer tools: testAuton(), tunePID()
  opticalAlign.cpp     # Goal alignment using the rear distance sensor

include/
  devices.h            # Extern declarations for all hardware objects
  auton_routes.h       # Auton function declarations
  auton_selector.h     # AutonSelector class declaration
  extended_chassis.h   # ExtendedChassis declaration
  old_systems.h        # drivePID declaration
  testing.h            # Testing declarations
  opticalAlign.h       # alignToLongGoal declarations
  averaged_imu.h       # AveragedIMU class (averages two IMUs for odometry)
  main.h               # PROS standard header + competition prototype declarations
```

---

## Hardware

### Drive
- 3-motor left side: ports 11, 13, 14 (blue cartridge, 600 RPM)
- 3-motor right side: ports 18, 17, 20
- 3.25 inch wheels, 11.5 inch track width

### Intake & Scoring
| Object | Port | Description |
|--------|------|-------------|
| `intake` | 2, 9 | Bottom roller — pulls game objects into robot |
| `intakeTop` | 10 | Upper roller — feeds balls into scoring mechanism |
| `smallIntake` | 19 | Auxiliary intake roller |

### Pneumatics (ADI Ports A–H)
| Port | Name | Function |
|------|------|----------|
| A | `lift` | Ball-scoring lift piston |
| B | `deScores` | De-scoring wings (push balls off goals) |
| C | `stopper` | Ball stopper (prevents rollback out of intake) |
| D | `loader` | Match-loader gate (releases balls from field wall) |

### Sensors
| Sensor | Port | Use |
|--------|------|-----|
| IMU (`imu`) | 3 | Primary inertial sensor for heading |
| IMU (`imu2`) | 16 | Secondary IMU — averaged with `imu` via `AveragedIMU` |
| Optical (`ballSensor`) | 19 | Ball color + proximity detection |
| Distance (`backDistance`) | 2 | Rear goal-alignment detection |

---

## Autonomous System

### Selecting a Route (Pre-Match)
When connected to a field control switch, `competition_initialize()` runs and shows the autonomous selector on the brain screen:

- **Left LCD button** — previous routine
- **Right LCD button** — next routine
- **Center LCD button** — toggle Red / Blue alliance color

### Competition Routines
Defined in `COMPETITION_ROUTINES[]` in `src/auton_selector.cpp`. To add a new routine, implement it in `auton_routes.cpp`, declare it in `auton_routes.h`, and add an entry to that array.

| Name | Function | Description |
|------|----------|-------------|
| Full AWP LEFT SIDE | `newfullLocalAWP` | Primary match route (left start, heading 69°) |
| Full FIELD SKILLS | `skillsNew` | 60-second skills run using odometry |
| Elim 9 Ball | `tylerAuton` | 9-ball elimination route (right start) |
| Odom AWP Right | `odomAWPHigh` | Right-side AWP with high-goal scoring |
| Left 7 Ball Push | `leftPush` | Left-side 7-ball push |
| Right 7 Ball Push Fast | `rightPushFast` | Fast right-side push |

### Coordinate System
LemLib uses a field-relative coordinate system:
- Origin = robot start position
- +X = right, +Y = forward
- θ (theta) = heading in degrees (0° = forward, clockwise positive)

### Motion Primitives
| Function | Source | Description |
|----------|--------|-------------|
| `drivePID(inches, timeout, kP)` | `old_systems.cpp` | Straight drive via motor encoders |
| `chassis.turnToHeading(deg, ms)` | LemLib | Turn to an absolute field heading |
| `chassis.moveToPose(x, y, θ, ms)` | LemLib | Curved path to a field pose using odometry |
| `drivePIDOdom(inches)` | `auton_routes.cpp` | Straight drive computed from current odometry pose |
| `outake(ms)` | `auton_routes.cpp` | Run all intakes in reverse for given duration |
| `intakeAll(ms)` | `auton_routes.cpp` | Run intake forward for given duration |
| `endSection(ms)` | `auton_routes.cpp` | Section timer + controller override for split testing |

---

## Driver Controls (Opcontrol)

| Input | Action |
|-------|--------|
| Left stick Y | Left drive motors |
| Right stick Y | Right drive motors |
| R1 | Intake forward (feed into scorer) |
| R1 + R2 | Full eject |
| R2 | Intake reverse |
| L1 (hold) | Stopper up (retain balls) |
| L2 (hold) | De-scoring wings out |
| DOWN (press) | Toggle lift |
| RIGHT (press) | Toggle loader (match-loader gate) |
| UP (press) | Toggle color sort on/off |

Joystick input passes through an **exponential curve** (x³ / 10000) for precise control at low speeds while preserving full power at max deflection.

---

## Testing Autonomous Routes (Without a Competition Switch)

`testAuton()` runs inside `opcontrol()` when not connected to a field switch:

1. Hold **A + X + Y** simultaneously on the controller to trigger the auton.
2. Optionally hold **L1** or **L2** before triggering to select an alternate branch.
3. Per-section timing is printed to the USB serial console for analysis.
4. Drive the robot back by hand and repeat.

To change which route runs, edit the active call (e.g. `rightPushFast(1)`) in `src/testing.cpp` inside `testAuton()`.

### Live PID Tuning
`tunePID()` in `testing.cpp` lets you adjust PID gains in real time:
- **Y** — cycle through kP → kI → kD
- **UP / DOWN** — multiply / divide the step size by 10
- **LEFT / RIGHT** — decrease / increase the selected gain

---

## Building & Flashing

```bash
# Build
pros make

# Flash to robot
pros upload

# Open serial monitor (for timing logs)
pros terminal
```
