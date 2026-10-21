# Plan 4 — Drive-to-pose and first autonomous routines

**Goal:** a reliable `DriveToPose` command (the building block of auto-align, auto-score and sensor-gated autos), a `DriveToNearestPose` wrapper, and the first PathPlanner autos — tested in simulation first.

**Why:** the most important thing to learn from top-team autos is that they are *not* a library of paths. 6328 builds every auto from `DriveToPose` goals sequenced by sensor conditions; 2056-style teams do the same with PID. Pre-planned paths are for long transits. The game-specific part (the list of target poses) is filled in at kickoff; everything else is reusable.

**Effort:** 1–2 weeks. `DriveToPose` is a day; PathPlanner setup is a day; the rest is tuning and sim testing.

**Prerequisites:** Plan 1 (PathPlanner config correct, odometry validated), Plan 3 (vision — a tag in view during auto keeps it on track).

---

## Step 1 — `DriveToPose` command (1 day)

YAMS already has `SwerveDrive.driveToPose(Pose2d)` using the PID controllers from `SwerveDriveSubsystem.java:42-43` (P=4 translation, P=1 rotation). It's fine for a quick demo but has no velocity profile, so it lunges. Build the profiled version modeled on 6328's.

Create `src/main/java/frc/robot/commands/DriveToPose.java`:

```java
package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.function.Supplier;

/** Drives to a field pose with a trapezoidal velocity profile. Goal is a supplier so it can move. */
public class DriveToPose extends Command {
  // Starting points from 6328's 2025 DriveToPose; retune on this robot.
  private static final double kDriveP = 1.8;
  private static final double kThetaP = 5.0, kThetaD = 0.5;
  private static final double kDriveToleranceMeters = 0.01;
  private static final double kThetaToleranceRad = Math.toRadians(1.0);
  private static final double kFfMinError = 0.01, kFfMaxError = 0.05;

  private final SwerveDriveSubsystem m_swerve;
  private final Supplier<Pose2d> m_goal;
  private final TrapezoidProfile m_driveProfile;
  private final PIDController m_driveController = new PIDController(kDriveP, 0, 0);
  private final ProfiledPIDController m_thetaController;

  private TrapezoidProfile.State m_driveState = new TrapezoidProfile.State();
  private Translation2d m_lastSetpointTranslation = Translation2d.kZero;

  public DriveToPose(
      SwerveDriveSubsystem swerve,
      Supplier<Pose2d> goal,
      double maxVelocity, double maxAccel,          // m/s, m/s²
      double maxAngularVelocity, double maxAngularAccel) {   // rad/s, rad/s²
    m_swerve = swerve;
    m_goal = goal;
    m_driveProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAccel));
    m_thetaController =
        new ProfiledPIDController(
            kThetaP, 0, kThetaD,
            new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAccel));
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_driveController.setTolerance(kDriveToleranceMeters);
    m_thetaController.setTolerance(kThetaToleranceRad);
    addRequirements(swerve);
  }

  /** Teleop-friendly defaults. Use a faster profile for auto. */
  public DriveToPose(SwerveDriveSubsystem swerve, Supplier<Pose2d> goal) {
    this(swerve, goal, 3.0, 3.0, Math.toRadians(360), Math.toRadians(720));
  }

  @Override
  public void initialize() {
    Pose2d current = m_swerve.getPose();
    ChassisSpeeds fieldVel = m_swerve.getFieldRelativeSpeed();
    Pose2d goal = m_goal.get();
    Translation2d toGoal = current.getTranslation().minus(goal.getTranslation());
    // Initial velocity along the line to the goal (negative = closing).
    double v0 = Math.min(0.0,
        -new Translation2d(fieldVel.vxMetersPerSecond, fieldVel.vyMetersPerSecond)
            .rotateBy(toGoal.getAngle().unaryMinus()).getX());
    m_driveState = new TrapezoidProfile.State(toGoal.getNorm(), v0);
    m_thetaController.reset(current.getRotation().getRadians(), fieldVel.omegaRadiansPerSecond);
    m_lastSetpointTranslation = current.getTranslation();
  }

  @Override
  public void execute() {
    Pose2d current = m_swerve.getPose();
    Pose2d goal = m_goal.get();
    Translation2d toGoal = current.getTranslation().minus(goal.getTranslation());
    double distance = toGoal.getNorm();

    // Fade feedforward out near the goal so the PID can finish without overshoot.
    double ffScale = MathUtil.clamp((distance - kFfMinError) / (kFfMaxError - kFfMinError), 0, 1);
    m_driveState = m_driveProfile.calculate(0.02, m_driveState, new TrapezoidProfile.State(0, 0));
    double driveVel =
        m_driveState.velocity * ffScale + m_driveController.calculate(distance, m_driveState.position);
    if (m_driveController.atSetpoint()) driveVel = 0;

    double thetaVel = m_thetaController.calculate(
        current.getRotation().getRadians(), goal.getRotation().getRadians());
    if (m_thetaController.atGoal()) thetaVel = 0;

    // Velocity points from current toward goal (negative along toGoal).
    Translation2d vel = new Translation2d(driveVel, 0).rotateBy(toGoal.getAngle()).unaryMinus();
    m_swerve.driveFieldRelative(new ChassisSpeeds(vel.getX(), vel.getY(), thetaVel));
  }

  @Override
  public boolean isFinished() {
    return m_driveController.atSetpoint() && m_thetaController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    m_swerve.driveFieldRelative(new ChassisSpeeds());
  }
}
```

Add to `SwerveDriveSubsystem` a plain (non-command) setter so commands can push speeds:

```java
public void driveFieldRelative(ChassisSpeeds speeds) {
  m_drive.setFieldRelativeChassisSpeeds(speeds);
}
public ChassisSpeeds getFieldRelativeSpeed() {
  return m_drive.getFieldRelativeSpeed();
}
```

Also raise YAMS's rotation controller from `new PIDController(1, 0, 0)` to around 3–5 so the built-in `driveToPose` is usable for quick tests.

## Step 2 — `DriveToNearestPose` (1 hour)

```java
public static Command driveToNearest(SwerveDriveSubsystem swerve, List<Pose2d> candidates) {
  return new DriveToPose(swerve, () -> swerve.getPose().nearest(candidates));
}
```

The `List<Pose2d>` is the only game-specific input. For now, put four practice poses in `Constants.FieldConstants` (e.g. 1 m in front of four tags) and bind `driveToNearest` to a driver button (e.g. A, `whileTrue`). Mirror poses for the red alliance with a single helper (Step 6).

## Step 3 — Tune `DriveToPose` (half a day on carpet)

1. Start at 1 m from a target pose. Watch the overshoot. If it overshoots, lower `kDriveP`; if it stalls short, raise it or lower `kFfMinError`.
2. Repeat from 3 m and from a diagonal. Repeat with the robot already moving (start it while driving).
3. Rotation: should settle in < 1 s with no hunting. Lower `kThetaD` if it hunts, raise `kThetaP` if it lags.
4. Record final gains in `Constants` and a note in `docs/drivetrain-measurements.md`.

**Friction compensation** (2910's trick): if the robot stalls 1–2 cm short, add a small constant (e.g. 0.05–0.1 m/s) in the direction of the error when `distance > tolerance`. Try `kFfMinError` first.

## Step 4 — PathPlanner autos (1 day setup)

### 4a. Robot-side setup in `RobotContainer`
Order matters: named commands must be registered **before** any auto or chooser is built.

```java
public RobotContainer() {
  // 1. Named commands (events used inside paths/autos).
  NamedCommands.registerCommand("print", Commands.print("marker hit"));
  // later: "intake", "score", etc.

  // 2. Then the chooser.
  m_autoChooser = AutoBuilder.buildAutoChooser();
  SmartDashboard.putData("Auto Chooser", m_autoChooser);

  // 3. Warm up the path follower so the first path doesn't stutter.
  FollowPathCommand.warmupCommand().schedule();

  configureBindings();
}
```

### 4b. Draw the first path
In the PathPlanner GUI, create `deploy/pathplanner/paths/Test-Line.path`: a 3 m straight line, then `Test-Square.path`. Create `deploy/pathplanner/autos/Test Square.auto` that runs the square. Commit the files.

### 4c. Run in simulation first
`./gradlew simulateJava` → pick the auto in the sim DS chooser → enable autonomous. In AdvantageScope's Field tab, overlay the planned path (`PathPlanner/activePath`) on the robot pose. The subsystem's `simulationPeriodic()` already calls `m_drive.simIterate()`. If the robot can't follow in sim, the gains or PathPlanner config are wrong — fix that before touching the robot.

### 4d. Run on carpet
Expect the real robot to track slightly worse than sim. The holonomic controller is `PIDConstants(5, 0, 0)` for both axes (`SwerveDriveSubsystem.java:67-68`); the community range is P 2–10 with a little D, no I. Tune with the square auto: if the robot cuts corners, raise translation P; if it wobbles, add D ≈ 0.1–0.3. (YAGSL's docs note PathPlanner's controller is independent of YAMS's drive-to-pose PIDs — don't try to make them match.)

### 4e. Pathfinding for long moves
`AutoBuilder.pathfindToPose(goal, new PathConstraints(3.0, 3.0, Math.toRadians(540), Math.toRadians(720)))` is already available because `AutoBuilder` is configured. Combine: pathfind to within ~1 m, then `DriveToPose` for the last meter. That combination is what 4915 found "extremely reliable".

## Step 5 — First real auto: the game-agnostic template (2 days)

Build it like 6328 — sequenced by sensors, not timers:

```java
public Command exampleAuto() {
  return Commands.sequence(
      Commands.runOnce(() -> m_swerve.resetOdometry(FieldConstants.START_LEFT)),
      new DriveToPose(m_swerve, () -> FieldConstants.SCORE_A, 4.0, 6.0, Math.toRadians(540), Math.toRadians(720)),
      // scoreCommand().withTimeout(1.0),  ← mechanism from Plan 6
      new DriveToPose(m_swerve, () -> FieldConstants.PICKUP_1),
      // intakeCommand().until(intake::hasPiece),
      new DriveToPose(m_swerve, () -> FieldConstants.SCORE_B)
  ).withTimeout(14.9);  // never let a command run into teleop
}
```

Register it in the chooser alongside PathPlanner autos: `m_autoChooser.addOption("Example", exampleAuto())`.

Design rules:
- Start poses must be exact: reset odometry from the vision estimate if a tag is visible at start; otherwise from the chosen start constant.
- Keep a tag in view for the whole auto. Re-plan camera placement if the auto spends > 2 s blind.
- Transition on sensors (`until(() -> hasPiece)`), with a timeout as a fallback, never on timers alone.
- Cut off everything at 14.9 s (2026 had a 20 s auto; adjust at kickoff).

## Step 6 — One alliance-flip helper

PathPlanner's flip supplier (`SwerveDriveSubsystem.java:70-72`) is evaluated when a path command *initializes*. A 2025 bug: autos built at boot, before the DS reports an alliance, didn't mirror. Isolate flipping in one place now, because the 2027 field-origin change will force a rewrite:

```java
public final class AllianceFlip {
  public static boolean isRed() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }
  public static Pose2d apply(Pose2d bluePose) {
    return isRed() ? FlippingUtil.flipFieldPose(bluePose) : bluePose;   // PathPlanner's util
  }
}
```

Use `AllianceFlip.apply(...)` for every `FieldConstants` pose at *runtime* (inside the supplier), never at construction. If you build autos at boot, wrap them in `Commands.deferredProxy(() -> exampleAuto())` so they're constructed when selected.

## Step 7 — Odometry validation before trusting any auto

Re-run Plan 1 Step 5c–5e after any drivetrain change. An auto cannot be more accurate than the odometry under it: "you're not going to get your robot to perform tenfold better in auto than it does in teleop".

---

## Done when

- [ ] `DriveToPose` drives to a pose from 3 m within 2 cm / 1° with no overshoot, including when started while moving.
- [ ] `driveToNearest` bound to a driver button with practice poses.
- [ ] Named commands registered before the chooser; `warmupCommand` scheduled.
- [ ] `Test Square.auto` runs in sim and on carpet, ending within 10 cm of start.
- [ ] One sensor-sequenced template auto in the chooser.
- [ ] `AllianceFlip` is the only place that reads the alliance for geometry.

## Risks

- An `EventTrigger`/named command that requires the drivetrain will interrupt the path. Mechanism commands must not require `SwerveDriveSubsystem`.
- Choreo: not now. Its time-optimal trajectories are 6–11% faster, but only once PathPlanner paths are accurate, and it faces a large 2027 rework (field-origin change). `PathPlannerPath.fromChoreoTrajectory()` lets you drop them in later.
- Wheel slip at 6 m/s² in auto: if the pose drifts after a hard launch, lower the auto accel or fix the COF in PathPlanner config.
