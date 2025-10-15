# BL Robotics 2025

![main workflow](https://github.com/BLlakers/2025_Official/actions/workflows/main.yml/badge.svg)

This repository contains our Java/WPILib robot code with a simulation-first workflow, a swerve drivetrain, vendor motor
abstractions, and CI with test coverage. This guide explains how to set up your environment, run the sim, deploy to the 
robot, and contribute.

---

## Table of Contents
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Project Layout](#project-layout)
- [Driver Station Controls](#driver-station-controls)
- [Running the Simulator](#running-the-simulator)
- [Deploying to the Robot](#deploying-to-the-robot)
- [Testing & Code Coverage](#testing--code-coverage)
- [Formatting & Static Checks](#formatting--static-checks)
- [Configuration & Constants](#configuration--constants)
- [Resources](#resources)
- [Contributing](#contributing)
- [License](#license)

---

## Prerequisites

- **OS:** Windows 10/11, macOS, or Linux
- **Java:** **JDK 17 (LTS)**
  ```bash
  java -version
  ```
- **IDE:** one of
    - **VS Code + WPILib Extension** (recommended)
    - IntelliJ IDEA / Eclipse (fine; use Gradle tasks)
- **Git:** for cloning & PRs
- **Gradle:** **not required** — project uses the **Gradle Wrapper** (`./gradlew`)

> Vendor libraries are managed via **vendordeps** JSON under `./vendordeps/`. GradleRIO & wrapper versions in `build.gradle` pin the WPILib toolchain.

---

## Quick Start

```bash
git clone <repo-url>
cd <repo-dir>

# First build (downloads WPILib & vendors)
./gradlew build

# Open the folder in your IDE (VS Code: install WPILib extension + Set Team Number)
```

---

## Project Layout

```
src/main/java/frc/robot/
  commands/            # Command-based routines (e.g., swervedrive/)
  subsystems/          # Subsystems (e.g., drivetrain/, elevator/, climb/)
  sim/                 # Simulation helpers (SwerveModuleSim, physics)
  support/             # Vendor abstractions (sparkmax/, sensors, utils)
  Constants.java       # Global constants (units in identifiers where possible)
vendordeps/            # Vendor JSONs (REV, PathPlanner, etc.)
```
---

## Driver Station Controls
Driver Station (3 Controllers):
```
USB 0: DRIVER CONTROLLER
├─ Swerve drive (sticks + triggers)
├─ Limelight tracking (A/X/Y buttons)
├─ Gyro reset (B button)
└─ Wheel lock (right stick)

USB 1: MANIPULATION/OPERATOR CONTROLLER
├─ Elevator positions (A/B/X/Y buttons)
├─ Algae mechanism (bumpers, back/start, POV, sticks)
├─ Coral mechanism (triggers, POV up)
└─ Climb (POV down)

USB 2: DEBUG CONTROLLER
├─ Manual elevator control (bumpers - limit switches)
├─ Algae mechanism overrides (A/B/X/Y buttons)
├─ Algae mechanism presets (POV, sticks)
└─ Additional test commands
```

### 1. **Driver Controller** (Channel 0)
- **Primary job:** Drive the robot
- **Secondary:** Limelight-based auto-alignment

### 2. **Manipulation Controller** (Channel 1)
- **Primary job:** Score game pieces
- Elevator preset positions (A/B/X/Y = different heights)
- Algae intake/mechanism control
- Coral manipulation
- Climb mechanism

### 3. **Debug Controller** (Channel 2)
- **Primary job:** Overrides and testing
- Manual elevator control (bypasses presets, uses limit switches)
- Algae mechanism testing/overrides
- Uses:
    - Practice/testing
    - When automated controls fail
    - Tuning/debugging on the field

## Running the Simulator

```bash
./gradlew simulateJava
```

This launches **WPILib Sim GUI**.

### Keyboard / Gamepad mapping
1. Sim GUI → **Driver Station → Joysticks**: Ensure **Keyboard** is **index 0**.
2. **Configure Keyboard**:
    - **Axis 0** ← A/D (strafe left/right)
    - **Axis 1** ← W/S (forward/back)
    - (Optional) **Axis 4** ← rotation
3. In simulation we read `DriverStation.getStickAxis(0, axis)`, so the above mappings immediately drive the robot.

### Sim behavior notes
- We **reset odometry on sim enable** so field-relative starts at heading ~0°.
- **Only one pose update per loop** in sim (no double counting between `periodic()` and `simulationPeriodic()`).
- Swerve modules use `SwerveModuleState.optimize(...)` and a snappy steering loop for crisp direction changes.

---

## Deploying to the Robot

Set your team number via VS Code WPILib (“Set Team Number”) or GradleRIO config.

```bash
# On the FRC network, robot powered
./gradlew deploy
```

Then enable using the **FRC Driver Station**.

---

## Testing & Code Coverage

### JUnit 5 + Mockito

- **JUnit 5** is enabled via `useJUnitPlatform()`.
- **Mockito** (including the JUnit 5 extension) is available for mocks/spies/captors.

Example test skeleton:

```java
@ExtendWith(org.mockito.junit.jupiter.MockitoExtension.class)
class ExampleTest {
  @org.mockito.Mock SomeDep dep;

  @org.junit.jupiter.api.Test
  void works() {
    // arrange
    // act
    // assert
  }
}
```

### JaCoCo coverage in Gradle

We enforce minimum coverage and generate HTML & XML reports.

```groovy
// build.gradle (Groovy DSL)
plugins {
  id 'java'
  id 'jacoco'
}

test {
  useJUnitPlatform()
}

jacoco {
  toolVersion = "0.8.10"
}

jacocoTestReport {
  dependsOn test
  reports {
    html.required = true   // build/reports/jacoco/test/html/index.html
    xml.required  = true   // build/reports/jacoco/test/jacocoTestReport.xml
    csv.required  = false
  }
}

jacocoTestCoverageVerification {
    dependsOn test
    violationRules {
        rule {
            limit {
                counter = 'LINE'
                value   = 'COVEREDRATIO'
                minimum = 0.03 // <-- adjust threshold as we expand our test coverage
            }
        }
    }
}

check.dependsOn jacocoTestReport, jacocoTestCoverageVerification
```

### GitHub Actions CI (build, test, upload coverage)

Create `.github/workflows/ci.yml`:

```yaml
name: FRC Team 2534 CI Pipeline
on:
  push:
    branches:
      - main
      - Dev
      - Testing
  pull_request:
jobs:

  build:
    runs-on: ubuntu-22.04
    container: wpilib/roborio-cross-ubuntu:2025-22.04
    steps:
      - uses: actions/checkout@v4

      - name: Add repository to git safe directories
        run: git config --global --add safe.directory $GITHUB_WORKSPACE

      - name: Grant execute permission for gradlew
        run: chmod +x gradlew

      - name: Compile and run tests on robot code
        run: ./gradlew --no-daemon clean check jacocoTestReport jacocoTestCoverageVerification

      - name: Upload JaCoCo HTML report
        uses: actions/upload-artifact@v4
        if: always()
        with:
          name: jacoco-html
          path: build/reports/jacoco/test/html

      - name: Upload JaCoCo XML (for tooling)
        uses: actions/upload-artifact@v4
        if: always()
        with:
          name: jacoco-xml
          path: build/reports/jacoco/test/jacocoTestReport.xml
```

> If coverage drops below the configured threshold, the CI job fails and the PR turns red. Reviewers can download the HTML report artifact and open `index.html`.

---

## Formatting & Static Checks

We use **Spotless** for formatting with a Palantir config.

```bash
# Auto-format
./gradlew spotlessApply

# Build + run tests + coverage gates
./gradlew check
```

---

## Configuration & Constants

- Keep **units in identifiers** (e.g., `MAX_SPEED_MPS`, `NOMINAL_BATT_VOLTS`, `VOLTS_PER_MPS`).
- Use `*Context` classes for tunables (PID gains, gear ratios, conversion factors).
- For REV SPARK MAX **Alternate Encoder** on the real robot, configure the **data port mode + CPR** *before* selecting `kAlternateOrExternalEncoder`. In **simulation**, prefer the **primary encoder** feedback unless you fully simulate the alt encoder.

---

## Resources

- **WPILib Docs:** https://docs.wpilib.org/
- **WPILib Java API:** https://first.wpi.edu/wpilib/allwpilib/docs/release/java/
- **Kinematics & Odometry (Swerve):** https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/index.html
- **Robot Simulation (WPILib):** https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/index.html
- **REV SPARK MAX Docs:** https://docs.revrobotics.com/sparkmax/
- **PathPlanner:** https://pathplanner.dev/
- **navX-MXP:** https://pdocs.kauailabs.com/navx-mxp/
- **Limelight:** https://docs.limelightvision.io/

---

## Contributing

- Branch from `main`.
- Keep changes focused; add/extend unit tests.
- Open a PR with:
    - **Why** + **What changed**
    - **Testing notes** (sim and/or real)
    - **Risks / required config**
- CI must pass (build, tests, coverage gates).

---

## License

This project follows the WPILib BSD license model unless otherwise noted in the repository.