# Plan 2 — Driver controls: input shaping, acceleration limiting, heading hold, Pigeon tilt

**Goal:** a drivetrain that feels precise at low speed, doesn't jerk a delicate game piece, drives straight without the driver correcting, and has a one-button slow mode. Then, as polish, uses the Pigeon 2.0's pitch/roll to back off when the robot starts to tip.

**Why:** the stream in `RobotContainer.java:32-39` is raw stick → speed with a 0.05 deadband. Every change of stick direction is an instantaneous acceleration step. Top teams shape inputs and limit the *velocity vector's* rate of change; a few also read tilt from the IMU.

**Effort:** 2–4 days. Steps 1–2 are an afternoon; Step 3 is the one that needs driver feedback; Step 5 is optional.

**Prerequisites:** Plan 1 Steps 1–3 (the robot drives straight and the steer loop is tuned). Driver available for feedback.

**Important API note:** this project's `SwerveInputStream` is `yams.mechanisms.swerve.utility.SwerveInputStream`, not the older `swervelib.SwerveInputStream` that most Chief Delphi posts and the broncbotz javadoc describe. Methods like `scaleTranslation`, `headingWhile`, `aimWhile`, `driveToPose` **do not exist** here. What does exist (verified with `javap` on YAMS 2026.10.03): `withDeadband`, `withScaleTranslation(double)`, `withScaleRotation(double)`, `withCubeTranslationControllerAxis()`, `withCubeRotationControllerAxis()`, `withAllianceRelativeControl()`, `withRobotRelative(BooleanSupplier)`, `withHeadingControl(BooleanSupplier)`, `withControllerHeadingAxis(x, y)`, `withAim(Supplier<Pose2d>, BooleanSupplier)`, `withTranslationOnly(BooleanSupplier)`, `withMaximumLinearVelocity`, `withMaximumAngularVelocity`, `clone()`. There is **no** slew or acceleration limiting on the stream — we add that as a wrapper.

---

## Step 1 — Input shaping (30 minutes)

Edit the stream in `RobotContainer.java`:

```java
private final SwerveInputStream m_driveStream =
    m_swerve
        .createInputStream(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX())
        .withDeadband(OperatorConstants.kDriveDeadband)   // raise to 0.10 in Constants
        .withCubeTranslationControllerAxis()                // fine control near center
        .withCubeRotationControllerAxis()
        .withScaleRotation(OperatorConstants.kRotationScale) // 0.6: full stick ≠ max spin
        .withAllianceRelativeControl();
```

`Constants.OperatorConstants`:

```java
public static final double kDriveDeadband = 0.10;
public static final double kRotationScale = 0.6;
public static final double kPrecisionTranslationScale = 0.3;
public static final double kPrecisionRotationScale = 0.4;
```

Cubing keeps full speed at full stick while making the first half of travel gentle. 6328 squares instead; try both, keep what the driver prefers.

## Step 2 — Precision mode (30 minutes)

The YAMS scale methods take fixed doubles, so build a second stream by cloning and pick between them with a supplier.

```java
private final SwerveInputStream m_precisionStream =
    m_driveStream
        .clone()
        .withScaleTranslation(OperatorConstants.kPrecisionTranslationScale)
        .withScaleRotation(OperatorConstants.kPrecisionRotationScale);
```

Change `SwerveDriveSubsystem.driveFieldOriented` to accept any `Supplier<ChassisSpeeds>` (a `SwerveInputStream` already is one):

```java
public Command driveFieldOriented(Supplier<ChassisSpeeds> speeds) {
  return run(() -> m_drive.setFieldRelativeChassisSpeeds(speeds.get())).withName("Drive");
}
```

Bind:

```java
Trigger precision = m_driverController.leftBumper();
m_swerve.setDefaultCommand(
    m_swerve.driveFieldOriented(
        () -> precision.getAsBoolean() ? m_precisionStream.get() : m_driveStream.get()));
```

## Step 3 — Acceleration limiter on the velocity vector (half a day + driver tuning)

This is the "delicate game piece" feature. Limit how fast the commanded `(vx, vy)` vector can change per 20 ms loop, preserving its direction, with a separate cap on rotation. Do **not** put two `SlewRateLimiter`s on the x and y axes: that distorts diagonal moves and, because the stream is field-relative, makes the limit depend on heading.

Create `src/main/java/frc/robot/util/ChassisAccelLimiter.java`:

```java
package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;

/**
 * Limits the per-loop change of a ChassisSpeeds so the robot accelerates smoothly. Translation is
 * limited as a vector (direction preserved); rotation is limited as a scalar. Limits are suppliers
 * so they can drop when a mechanism is raised or a fragile piece is held.
 */
public class ChassisAccelLimiter {
  private static final double kLoopSeconds = 0.02;

  private final DoubleSupplier m_maxLinearAccel;   // m/s²
  private final DoubleSupplier m_maxLinearDecel;   // m/s², usually >= accel so the driver can stop
  private final DoubleSupplier m_maxAngularAccel;  // rad/s²

  private Translation2d m_prevLinear = Translation2d.kZero;
  private double m_prevOmega = 0.0;

  public ChassisAccelLimiter(
      DoubleSupplier maxLinearAccel, DoubleSupplier maxLinearDecel, DoubleSupplier maxAngularAccel) {
    m_maxLinearAccel = maxLinearAccel;
    m_maxLinearDecel = maxLinearDecel;
    m_maxAngularAccel = maxAngularAccel;
  }

  /** Call when the drive command starts so the limiter doesn't fight an already-moving robot. */
  public void reset(ChassisSpeeds current) {
    m_prevLinear = new Translation2d(current.vxMetersPerSecond, current.vyMetersPerSecond);
    m_prevOmega = current.omegaRadiansPerSecond;
  }

  public ChassisSpeeds calculate(ChassisSpeeds desired) {
    Translation2d want = new Translation2d(desired.vxMetersPerSecond, desired.vyMetersPerSecond);
    Translation2d delta = want.minus(m_prevLinear);

    // Slowing down (new magnitude smaller) gets the decel limit; otherwise the accel limit.
    boolean slowing = want.getNorm() < m_prevLinear.getNorm();
    double maxStep = (slowing ? m_maxLinearDecel : m_maxLinearAccel).getAsDouble() * kLoopSeconds;
    if (delta.getNorm() > maxStep) {
      delta = delta.times(maxStep / delta.getNorm());
    }
    m_prevLinear = m_prevLinear.plus(delta);

    double omegaStep = m_maxAngularAccel.getAsDouble() * kLoopSeconds;
    double dOmega = desired.omegaRadiansPerSecond - m_prevOmega;
    m_prevOmega += Math.copySign(Math.min(Math.abs(dOmega), omegaStep), dOmega);

    return new ChassisSpeeds(m_prevLinear.getX(), m_prevLinear.getY(), m_prevOmega);
  }
}
```

Wire it into the subsystem so the reset happens automatically. Add to `SwerveDriveSubsystem`:

```java
private final ChassisAccelLimiter m_limiter =
    new ChassisAccelLimiter(
        () -> DriveConstants.kMaxLinearAccel,
        () -> DriveConstants.kMaxLinearDecel,
        () -> DriveConstants.kMaxAngularAccel);

/** Field-relative drive with acceleration limiting; use this as the default command. */
public Command driveFieldOrientedSmooth(Supplier<ChassisSpeeds> speeds) {
  return runOnce(() -> m_limiter.reset(m_drive.getFieldRelativeSpeed()))
      .andThen(run(() -> m_drive.setFieldRelativeChassisSpeeds(m_limiter.calculate(speeds.get()))))
      .withName("Drive (smooth)");
}
```

`Constants.DriveConstants` *(starting points — tune with the driver)*:

```java
public static final double kMaxLinearAccel = 3.0;   // m/s²
public static final double kMaxLinearDecel = 4.0;   // m/s²
public static final double kMaxAngularAccel = 12.0; // rad/s²
```

### Tuning procedure
Start deliberately low (2 m/s²) and raise until one of these breaks first — that's your limit:
1. Wheels slip on launch (watch module velocity vs chassis velocity in the log).
2. Brownout (battery voltage dips below ~7 V in the log — Plan 5).
3. The robot rocks on its wheels / a mechanism whips.
4. The driver says it feels sluggish.

Put the "fragile piece" value in a separate constant and swap the supplier when that state is active (Plan 6 gives you the state; until then, a bumper button is fine):

```java
() -> m_holdingFragile.getAsBoolean() ? DriveConstants.kFragileLinearAccel : DriveConstants.kMaxLinearAccel
```

### Mechanism-height scaling (for later)
When there is an elevator, 6328 scales *rotation* to 20% when extended rather than slew-limiting. Do that on the stream output: `speeds.times(k)` is wrong for rotation only, so build a new `ChassisSpeeds(vx, vy, omega * k)`.

## Step 4 — Heading hold (2–3 hours)

When the driver isn't rotating, hold the last heading with a PID so bumps and uneven wheel wear don't yaw the robot. YAMS's `withHeadingControl` drives *to a stick-pointed heading*; it does not "hold the last heading when idle", so this is another wrapper.

Create `src/main/java/frc/robot/util/HeadingHold.java`:

```java
package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/** Holds the robot's heading when the driver releases the rotation stick. */
public class HeadingHold {
  private static final double kRotationDeadband = 0.05;  // rad/s, after input shaping
  private static final double kLatchDelaySeconds = 0.2;  // let the robot settle before latching

  // Starting point from the AdvantageKit swerve template.
  private final ProfiledPIDController m_pid =
      new ProfiledPIDController(5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
  private final Supplier<Rotation2d> m_heading;
  private final Timer m_sinceRelease = new Timer();
  private Rotation2d m_target = null;

  public HeadingHold(Supplier<Rotation2d> heading) {
    m_heading = heading;
    m_pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void reset() {
    m_target = null;
    m_pid.reset(m_heading.get().getRadians());
    m_sinceRelease.restart();
  }

  public ChassisSpeeds calculate(ChassisSpeeds desired) {
    if (Math.abs(desired.omegaRadiansPerSecond) > kRotationDeadband) {
      m_target = null;             // driver is steering
      m_sinceRelease.restart();
      return desired;
    }
    if (m_target == null) {
      if (!m_sinceRelease.hasElapsed(kLatchDelaySeconds)) {
        return desired;            // still coasting after release
      }
      m_target = m_heading.get();
      m_pid.reset(m_target.getRadians());
    }
    double omega = m_pid.calculate(m_heading.get().getRadians(), m_target.getRadians());
    return new ChassisSpeeds(desired.vxMetersPerSecond, desired.vyMetersPerSecond, omega);
  }
}
```

Compose it in `driveFieldOrientedSmooth`: limiter first, then heading hold (so the hold's correction isn't rate-limited):

```java
m_drive.setFieldRelativeChassisSpeeds(m_headingHold.calculate(m_limiter.calculate(speeds.get())))
```

and reset both in the `runOnce`. Use `() -> m_drive.getPose().getRotation()` as the heading source — the estimator's rotation, not raw gyro yaw, so it stays correct after an odometry reset.

Disable heading hold while `lock()` or any auto-align runs (they own rotation).

## Step 5 — Pigeon 2.0: calibrate, trim, and (optionally) tilt-aware limiting

### 5a. Mount calibration (15 minutes, do once after the Pigeon's mount is final)
Phoenix Tuner X → the Pigeon → "Calibrate". Robot level and stationary. This sets `MountPoseConfigs` so pitch/roll/yaw are 0 when flat. Without it, tilt readings are offset and useless for Step 5c.

### 5b. Yaw trim (only if Plan 1 Step 5c showed consistent drift)
Several teams measured 1.5–2° of yaw error per full rotation. If the 10-rotation test showed a consistent error, set `GyroTrimConfigs.GyroScalarZ` in Tuner X or in code. Don't touch this unless the error is repeatable in the same direction.

### 5c. Rules for yaw in code
- Pre-match squaring with the existing `zeroGyro` on Start is fine.
- Never "fix" field orientation mid-match with `pigeon.setYaw()`. Use `SwerveDrive.resetOdometry(Pose2d)` and read heading from `getPose().getRotation()`.

### 5d. Optional: tilt-aware acceleration limiting
YAGSL only consumes yaw. For pitch/roll, make your own `Pigeon2` object on the same CAN ID (reading signals from two objects on one device is fine).

```java
private final Pigeon2 m_pigeon = new Pigeon2(42);
private final StatusSignal<Angle> m_pitch = m_pigeon.getPitch();
private final StatusSignal<Angle> m_roll = m_pigeon.getRoll();

// In the constructor: 50 Hz is plenty for tilt and keeps CAN utilization down.
BaseStatusSignal.setUpdateFrequencyForAll(50, m_pitch, m_roll);

/** 1.0 when flat, falling toward 0 as tilt grows past the threshold. */
public double tiltScale() {
  BaseStatusSignal.refreshAll(m_pitch, m_roll);
  double tilt = Math.hypot(m_pitch.getValueAsDouble(), m_roll.getValueAsDouble()); // degrees
  double over = Math.max(0, tilt - DriveConstants.kTiltThresholdDeg);
  return MathUtil.clamp(1.0 - over / DriveConstants.kTiltFullStopDeg, 0.25, 1.0);
}
```

Feed it into the limiter's supplier: `() -> DriveConstants.kMaxLinearAccel * tiltScale()`. Starting points: threshold 3°, full-stop range 8°. Only a handful of teams do this; acceleration limiting (Step 3) does most of the work. Treat this as polish and test it on blocks first (tip the robot by hand, watch `tiltScale()` in the log).

Keep CAN utilization under ~80% (Driver Station → CAN tab). Eight motor controllers plus a Pigeon at default rates is fine; don't raise signal rates above 100 Hz on the roboRIO bus.

---

## Done when

- [ ] Deadband 0.10, cubed inputs, rotation scaled; driver approves.
- [ ] Left bumper precision mode at 30%/40%.
- [ ] `ChassisAccelLimiter` in the default drive command with tuned limits and a lower "fragile" limit selectable by a button.
- [ ] Heading hold latches ~0.2 s after stick release; robot drives a straight line hands-off-rotation across the field.
- [ ] Pigeon mount-calibrated in Tuner X; drift re-checked.
- [ ] (Optional) tilt scale logged and verified on blocks.

## Risks

- Heading hold fights `lock()` and auto-align — make sure those commands require the subsystem so the default command is interrupted.
- A decel limit that's too low means the robot can't stop fast enough. Keep decel ≥ accel.
- Cubed *rotation* can feel dead; if so, use `withScaleRotation` alone.
- Reading the Pigeon from two objects: configure (`getConfigurator().apply(...)`) from only one place — YAGSL's. Don't apply configs from the second object.
