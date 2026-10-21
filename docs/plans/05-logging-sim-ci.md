# Plan 5 — Logging, simulation habit, and CI

**Goal:** three habits every top-team repo has and this one lacks: you can *see* what the robot did (logging + AdvantageScope), you can *test without the robot* (simulation), and the code *can't rot* (CI, formatter, branch protection).

**Why do this early:** Plans 1–4 all involve tuning, and tuning without logs is guessing. CI takes an hour and catches every "it compiled on my laptop" from then on.

**Effort:** about a day each. Do CI first (it's the shortest), then logging, then sim.

**Prerequisites:** none. Can run in parallel with Plan 1 on a different student.

---

## Part A — CI, formatting, and branch protection (2–3 hours)

### A1. Spotless (auto-formatting)
In `build.gradle`, add the plugin and config:

```groovy
plugins {
    id "java"
    id "edu.wpi.first.GradleRIO" version "2026.2.1"
    id "com.diffplug.spotless" version "7.0.2"
}

spotless {
    java {
        target fileTree(".") {
            include "**/*.java"
            exclude "**/build/**", "**/build-*/**"
        }
        toggleOffOn()
        googleJavaFormat()
        removeUnusedImports()
        trimTrailingWhitespace()
        endWithNewline()
    }
    groovyGradle {
        target fileTree(".") { include "**/*.gradle"; exclude "**/build/**" }
        greclipse()
        trimTrailingWhitespace()
        endWithNewline()
    }
}
```

Run `./gradlew spotlessApply` once and commit the result as its own "Apply formatter" commit. From then on, `./gradlew build` runs `spotlessCheck`; students run `spotlessApply` before pushing. (The existing code already follows Google Java style, so the diff should be small.)

### A2. GitHub Actions
Create `.github/workflows/build.yml`:

```yaml
name: Build

on:
  push:
    branches: [main]
  pull_request:

jobs:
  build:
    runs-on: ubuntu-latest
    container: wpilib/roborio-cross-ubuntu:2026-22.04
    steps:
      - uses: actions/checkout@v4
      - name: Grant execute permission
        run: chmod +x gradlew
      - name: Build and test
        run: ./gradlew build --no-daemon
```

`./gradlew build` compiles for both roboRIO and desktop, runs `spotlessCheck`, and runs JUnit tests (Plan 7). Push, confirm a green check.

### A3. Branch protection
GitHub → Settings → Branches → add rule for `main`: require a pull request, require the "Build" status check, (optionally) require one approval. Students work on branches named `feature/<thing>`; a mentor or lead student merges.

### A4. Conventions doc
Create `docs/conventions.md` (half a page): branch naming, "run `spotlessApply` before pushing", "test in sim before asking for review", where constants go, how to name NetworkTables keys (`Subsystem/Name`).

## Part B — Logging (half a day)

### Which library
| | Effort | Tunables | Replay | Works with YAGSL/YAMS |
|---|---|---|---|---|
| **Epilogue** (WPILib, built in) | lowest — `@Logged` annotations | no | no | yes |
| **DogLog** (vendordep) | low — `DogLog.log(key, value)` | yes | no | yes |
| AdvantageKit | high — IO layers | yes | yes | **replay is incompatible** with YAGSL (its docs say so); logging-only is possible but pointless |

**Recommendation: Epilogue now.** `build.gradle` already has `annotationProcessor wpi.java.deps.wpilibAnnotations()`, so it's zero-install. Switch to DogLog later if you want live tunable constants. Don't adopt AdvantageKit while on YAGSL.

### B1. Turn on Epilogue and the data log
In `Robot.java`:

```java
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

@Logged
public class Robot extends TimedRobot {
  private final RobotContainer m_robotContainer;

  public Robot() {
    DataLogManager.start();                       // writes /home/lvuser/logs or a USB stick
    DriverStation.startDataLog(DataLogManager.getLog()); // joystick + DS data in the log
    m_robotContainer = new RobotContainer();
    Epilogue.bind(this);                          // logs every @Logged field each loop
  }
  ...
}
```

Annotate `RobotContainer`, `SwerveDriveSubsystem`, and (later) `Vision` with `@Logged`. Epilogue walks the object graph and logs public fields/getters of supported types (numbers, booleans, `Pose2d`, `ChassisSpeeds`, arrays). Add getters for anything you want to see, e.g. in `SwerveDriveSubsystem`:

```java
@Logged(name = "FieldRelativeSpeed")
public ChassisSpeeds getFieldRelativeSpeed() { ... }
```

Exclude noisy or unsupported fields with `@NotLogged`. Default config logs to NetworkTables *and* the data log; for competition, set `Epilogue.configure(c -> c.backend = ...)` to the data-log-only backend to save bandwidth (see WPILib's Epilogue docs for the exact config call).

YAMS already publishes `swerve/...` telemetry and writes it to the DataLog when `updateTelemetry()` runs, so the drivetrain is covered without extra work.

### B2. Log the things the other plans need
Add explicit log points for: battery voltage (`RobotController.getBatteryVoltage()`), CAN utilization, the acceleration limiter's input vs output (Plan 2), vision accepted/rejected counts and per-camera poses (Plan 3), and `DriveToPose` setpoint vs measured (Plan 4).

### B3. AdvantageScope
Install from the WPILib tools. Open a `.wpilog` from the roboRIO (`/home/lvuser/logs`, or a USB stick plugged into the RIO — recommended, logs fill the RIO's small disk). Save a layout `docs/advantagescope/drivetrain.json` with: Field tab (robot pose + vision poses), Line graph (module velocity setpoint vs measured), Line graph (battery voltage). Commit it so everyone opens the same view.

### B4. Elastic dashboard
Use Elastic (WPILib tools → Elastic) for the driver dashboard, not Shuffleboard (removed in 2027). Put the auto chooser, field, battery voltage, and camera streams on it. Save the layout to `src/main/deploy/elastic-layout.json` so it deploys with the code and can be loaded from the robot.

Keep the auto chooser creation (`RobotContainer.java:46-47`) in one small method — `SendableChooser` is replaced in 2027 (Plan 8).

## Part C — Simulation habit (half a day)

The swerve already simulates: `SwerveDriveSubsystem.simulationPeriodic()` calls `m_drive.simIterate()`, and `pidfproperties_sim.json` overrides gains in sim.

### C1. Make sim the default test loop
1. `./gradlew simulateJava` (or WPILib: Simulate Robot Code). The sim GUI opens with a DS panel; map a gamepad in the Joysticks pane.
2. Open AdvantageScope → connect to the simulator → Field tab. You should be able to drive the robot around the field.
3. Document this in `docs/conventions.md` as "before you ask for review, drive it in sim".

### C2. Sim-specific gains
If sim behaves differently from the real robot (it will — the physics are idealized), tune `pidfproperties_sim.json` separately rather than compromising the real gains. Also note YAGSL's advice to disable heading correction / cosine compensation in sim if the sim robot twitches.

### C3. Vision in sim (after Plan 3)
PhotonLib's `VisionSystemSim` + `PhotonCameraSim` can render fake AprilTag detections from the sim pose. YAGSL-Example's `Vision.java` has a working pattern. Add it when Plan 3 is done so autos can be tested with vision in sim.

### C4. Mechanism sim (with Plan 6)
YAMS mechanisms (`Elevator`, `Arm`, `Pivot`, …) ship with `simIterate()` and AdvantageScope mechanism visualization. The first mechanism you write should be driven in sim before any hardware exists.

---

## Done when

- [ ] `./gradlew build` passes locally and in GitHub Actions on every PR.
- [ ] Spotless applied; `main` protected.
- [ ] `docs/conventions.md` exists.
- [ ] Epilogue + DataLogManager on; `.wpilog` files opened in AdvantageScope with a committed layout.
- [ ] Elastic layout in `deploy/`.
- [ ] Every student has driven the robot in sim.

## Risks

- Logging to the RIO's internal disk fills it. Use a USB stick (`DataLogManager` prefers `/u` automatically) and delete old logs at events.
- Epilogue logs every loop; a huge object graph costs loop time. Watch `Epilogue` overhead in the log and `@NotLogged` what you don't need.
- `spotlessCheck` failing CI confuses new students — the fix is always `./gradlew spotlessApply`. Put that in the conventions doc in bold.
