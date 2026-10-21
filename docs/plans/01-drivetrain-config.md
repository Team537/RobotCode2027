# Plan 1 — Drivetrain configuration and validation

**Goal:** a drivetrain whose odometry can be trusted, whose motors are protected, and whose PathPlanner physics match the real robot — so that vision, drive-to-pose and autos (Plans 3–4) are built on solid ground.

**Why first:** every later plan assumes the swerve reports velocity honestly and tracks where it is. The current config has placeholder physics, an untuned steer loop, and a Kraken/NEO drive-motor mix that no public team code runs.

**Effort:** 1–2 days of config and code, plus two or three sessions on carpet with the robot.

**Prerequisites:** a charged battery, the robot on carpet, a laptop with WPILib 2026 and AdvantageScope or Elastic, and a tape measure.

---

## Step 0 — Verify the hardware and decide on the motor mix

The JSON and the README disagree:

- `src/main/deploy/swerve/README.md` says "REV MAXSwerve modules".
- `deploy/swerve/base/modules/frontleft.json` and `frontright.json` say `talonfx_krakenx60` (CAN 2, 4).
- `backleft.json` and `backright.json` say `sparkmax_neo` (CAN 6, 8).

1. Open the robot. Write down, per corner: drive motor model and controller, steer motor, encoder type, CAN ID. Check CAN IDs in Phoenix Tuner X (Krakens) and REV Hardware Client (SparkMaxes).
2. Fix every JSON file that disagrees with reality.
3. **Decide:** keep the mix or match all four drive motors.

| Option | Pros | Cons |
|---|---|---|
| Four matching motors (recommended) | One PID file is correct for all modules; PathPlanner's motor type is accurate; no risk of front/rear fighting | Cost of two motors + controllers |
| Keep Kraken front / NEO rear | No hardware spend | Chassis capped at NEO speed; one shared `pidfproperties.json` serves two different vendor velocity loops; front can out-torque rear and yaw the chassis under full throttle; no public precedent to copy |

If you keep the mix, follow the "Mixed-motor extras" at the end of this plan.

## Step 1 — Make current limits explicit

YAGSL's `physicalproperties.json` schema supports `statorCurrentLimit` (defaults: drive 40 A, angle 20 A). Make them explicit so nobody has to guess, and so the Kraken limit is deliberate.

Edit `src/main/deploy/swerve/base/modules/physicalproperties.json`:

```json
{
  "$schema": "https://config.yagsl.com/schemas/physicalproperties.json",
  "gearing": {
    "drive": { "gearRatio": 4.71, "diameter": 3 },
    "angle": { "gearRatio": 46.42 }
  },
  "statorCurrentLimit": {
    "drive": 40,
    "angle": 20
  }
}
```

- **Steer 20 A** is not negotiable for NEO 550s: REV's locked-rotor test shows a NEO 550 survives 220 s at 20 A, ~27 s at 40 A, and ~2 s at the 80 A default. A stalled steer motor at 40 A+ is a dead motor in one match.
- **Drive 40 A** is conservative. Raise to 60 A once the robot drives well and you've checked for brownouts (Plan 5's logging makes this visible). CTRE's reference uses 120 A slip current for Krakens; don't go there with mixed motors.

## Step 2 — Set the chassis speed limits and discretization in code

`SwerveDriveSubsystem.java:38-44` sets no max speed and no discretization. Add both.

```java
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

SwerveDriveConfig config =
    new SwerveDriveConfig()
        .withSubsystem(this)
        .withStartingPose(new Pose2d(3, 3, Rotation2d.kZero))
        .withTranslationController(new PIDController(4, 0, 0))
        .withRotationController(new PIDController(1, 0, 0))
        // Max module speed = free speed of the SLOWEST drive motor on the robot.
        // NEO: 5676 rpm / 4.71 on a 3 in wheel ≈ 4.81 m/s. Kraken ≈ 5.08 m/s.
        .withMaximumModuleSpeed(MetersPerSecond.of(DriveConstants.kMaxModuleSpeedMps))
        // Chassis limits: ~85% of free speed is what the robot can actually hold under load.
        .withMaximumChassisSpeed(
            MetersPerSecond.of(DriveConstants.kMaxChassisSpeedMps),
            DegreesPerSecond.of(DriveConstants.kMaxAngularSpeedDps))
        // Fixes the sideways "skew" when translating and rotating at the same time.
        .withDiscretizationTime(Seconds.of(0.02))
        .withTelemetry("swerve", new SwerveDriveTelemetryConfig(TelemetryVerbosity.HIGH));
```

Add to `Constants.java`:

```java
public static class DriveConstants {
  /** Free speed of the slowest drive motor (NEO) through 4.71:1 onto a 3 in wheel. */
  public static final double kMaxModuleSpeedMps = 4.81;
  /** Replace with the measured value from Step 5. 85% of free speed is a safe start. */
  public static final double kMaxChassisSpeedMps = 4.1;
  public static final double kMaxAngularSpeedDps = 540.0;
}
```

`withMaximumChassisSpeed`, `withMaximumModuleSpeed` and `withDiscretizationTime(Time)` are all on `yams.mechanisms.config.SwerveDriveConfig` in 2026.10.03 (verified with `javap`).

## Step 3 — Tune the steer loop

`pidfproperties.json` has steer `p: 0.01`, which the README itself flags as "likely far too low". Under-gained steering shows up as the robot twisting while driving straight.

Procedure (robot on blocks, wheels free):

1. Deploy. In AdvantageScope, plot the `swerve/.../<module>` angle setpoint and measured angle (YAMS publishes these under the `swerve` telemetry key at HIGH verbosity).
2. Drive the sticks in a slow circle so the modules sweep. Watch how far measured lags setpoint.
3. Double `angle.p` and redeploy. Repeat until the module overshoots or buzzes, then halve once.
4. If it buzzes at rest, add a small `d` (start at 1/100 of `p`).
5. Copy the final values into `pidfproperties_sim.json` only if sim behaves the same; otherwise leave sim with its own gains (that file exists for exactly this reason).

Expected outcome: a module snaps to a new setpoint in well under 0.25 s with at most one small overshoot.

Then the drive loop:

6. With `drive.p = 0.9, d = 0.3` the robot should hold a commanded velocity. Plot desired vs measured module velocity while driving at half stick. If measured lags by more than ~10%, raise `p`; if it hunts, lower it.
7. With mixed motors this one `p` governs two different vendor loops — see "Mixed-motor extras".

## Step 4 — Fix PathPlanner's robot config

`src/main/deploy/pathplanner/settings.json` is read at startup by `RobotConfig.fromGUISettings()` (`SwerveDriveSubsystem.java:56`). Since PathPlanner 2025.1, every path's acceleration limit comes from these numbers, so wrong values generate paths the robot can't follow.

Open the project in the PathPlanner GUI (Settings tab) and set:

| Field | Set to | How to get it |
|---|---|---|
| Robot mass | measured | Bathroom scale, robot with bumpers and battery |
| Robot MOI | measured or estimated | Rough: `mass * (length² + width²) / 12` using bumper-to-bumper dims. Better: SysId angular vs linear kA ratio once Plan 5 logging exists |
| Bumper width/length | measured | Bumper-to-bumper |
| Wheel radius | 0.0381 m | Already correct (3 in). Must match YAGSL's `diameter: 3` |
| Drive gearing | 4.71 | Already correct |
| True max drive speed | from Step 5 | Until measured, use 4.1 |
| Drive motor type | NEO | The weaker motor; PathPlanner supports only one type |
| Drive current limit | 40 | Match `physicalproperties.json` |
| Wheel COF | 1.2 | Vendor figure for MAXSwerve wheels; lower it if wheels slip in Step 5 |
| Module positions | ±0.254 m | Already matches `location: ±10 in` in the module JSON |

Commit the regenerated `settings.json`.

## Step 5 — Carpet validation tests

Do these in order. Each one takes 10–20 minutes. Record the numbers in `docs/drivetrain-measurements.md` so next year's students have them.

### 5a. Encoder sanity
Push the robot forward 1 m by hand with the code enabled. In AdvantageScope the pose `x` should increase by ~1.0 m and every module position should increase (not decrease, not stay flat). Fix any inverted module in its JSON.

### 5b. Wheel-radius check (rotate in place)
Mark the heading. Command a slow spin for exactly 10 full rotations (count with the gyro, not by eye). Compare the integrated module distance to `10 × 2π × (distance from center to module)`. If odometry says the wheels rolled too far, the configured wheel diameter is too large; adjust `diameter` in `physicalproperties.json` and `driveWheelRadius` in PathPlanner together. Worn wheels are usually 1–3% smaller than nominal.

### 5c. 10 m straight line
Reset odometry at a start mark. Drive straight 10 m (tape measure on the floor). Odometry error should be under 10 cm (1%) and heading drift under 2°. If heading drifts the same direction every time, Pigeon trim is the fix (Plan 2, Step 4).

### 5d. Max speed
Full stick for 3 s on a long straight. Read the plateau velocity from the logged `getRobotRelativeSpeed()` / module velocities. Put that number in `kMaxChassisSpeedMps` and PathPlanner's "True max drive speed".

### 5e. Square drive
Drive a 2 m square by hand and return to the start mark. Final pose error tells you the combined odometry quality you're handing to Plan 3. Under 15 cm is good enough to start vision work.

## Step 6 — Clean up

- Delete `src/main/deploy/example.txt` (template leftover).
- Update `src/main/deploy/swerve/README.md` so it describes the real hardware and records tuned values.
- Commit with a message that lists the measured numbers.

---

## Done when

- [ ] Every module JSON matches the physical robot (motor type, CAN ID, encoder).
- [ ] `physicalproperties.json` has explicit stator limits (angle 20 A).
- [ ] `SwerveDriveSubsystem` sets max module speed, max chassis speed and discretization.
- [ ] Steer modules snap to setpoint without oscillation; drive velocity tracks within ~10%.
- [ ] PathPlanner `settings.json` has measured mass, MOI, bumpers, true max speed, motor type = weakest motor.
- [ ] 10 m line test: < 10 cm position error, < 2° heading drift.
- [ ] Measurements recorded in `docs/drivetrain-measurements.md`.

## Risks and gotchas

- **Changing an earlier step invalidates later ones.** YAGSL's drift guide orders tuning as: module PIDs → vision std devs → discretization → gyro scale factor. If you re-tune steer P later, re-run the carpet tests.
- **Wheel diameter lives in two places** (YAGSL JSON and PathPlanner settings). They must agree or odometry and paths disagree.
- **Hard floors lie.** Carpet only for 5b–5e.
- **Kraken IDs 2/4 and NEO IDs 6/8 on the same bus.** If a Kraken doesn't appear in Tuner X, it's probably on the CANivore bus; `canbus: ""` means the roboRIO bus.

## Mixed-motor extras (only if you keep Kraken front / NEO rear)

1. Cap the chassis at the NEO-limited speed (done in Step 2). The Krakens will never be asked for more than the NEOs can give.
2. The shared `drive.p` is applied by YAMS to both a TalonFX and a SparkMax velocity loop. Whether YAMS normalizes gains across vendors is **not verified**. Tune so the *rear* (NEO) modules track; if the front then oscillates, lower `p` and accept slightly looser tracking.
3. Watch for yaw under hard acceleration: if the robot consistently turns one way when you floor it in a straight line, the front is out-pulling the rear. Lower the drive stator limit (e.g. 30 A) and re-test; if that's not enough, this is the signal to match motors.
4. Expect SysId feedforward (kS/kV) to differ per motor type. YAGSL's JSON does not have per-module PID files, so you cannot express that today — one more reason to match motors.
