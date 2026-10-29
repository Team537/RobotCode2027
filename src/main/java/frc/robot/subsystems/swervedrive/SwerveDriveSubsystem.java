// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.ChassisAccelLimiter;
import frc.robot.util.HeadingHold;
import java.io.File;
import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;

/**
 * Swerve drivetrain. YAGSL builds the hardware from the JSON config in deploy/swerve/base.
 *
 * <p>Temporarily on YAGSL 2026.3.12 because the newer YAGSL/YAMS sends feedforward parameters that
 * 2025 (25.x) Spark MAX firmware rejects with "Invalid parameter id". Move back to YAGSL/YAMS once
 * the steer Spark MAXes are on 26.x firmware.
 */
public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveDrive m_drive;
  private final HeadingHold m_headingHold = new HeadingHold(() -> getPose().getRotation());

  /** Creates the drivetrain from the YAGSL config in deploy/swerve/base. */
  public SwerveDriveSubsystem() {
    try {
      m_drive =
          new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/base"))
              .createSwerveDrive(DriveConstants.kMaxModuleSpeedMps, new Pose2d(3, 3, Rotation2d.kZero));
    } catch (IOException e) {
      throw new RuntimeException("Failed to load YAGSL swerve config", e);
    }
    m_drive.setMaximumAllowableSpeeds(
        DriveConstants.kMaxChassisSpeedMps,
        Units.degreesToRadians(DriveConstants.kMaxAngularSpeedDps));
    // Stops the sideways drift when translating and rotating at the same time.
    m_drive.setChassisDiscretization(true, 0.02);
    // The accel limiter and HeadingHold own heading control, so YAGSL's own correction stays off.
    m_drive.setHeadingCorrection(false);

    configurePathPlanner();
  }

  /** Configures PathPlanner's {@link AutoBuilder} so autos and paths can drive this subsystem. */
  private void configurePathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e) {
      throw new RuntimeException("Failed to load PathPlanner GUI settings", e);
    }

    AutoBuilder.configure(
        m_drive::getPose,
        m_drive::resetOdometry,
        m_drive::getRobotVelocity,
        (speeds, feedforwards) -> m_drive.drive(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        config,
        () ->
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Red,
        this);
  }

  /**
   * Creates a joystick input stream for this drivetrain. Inputs should be in [-1, 1] with
   * positive x forward, positive y left and positive rotation counterclockwise.
   *
   * @param x Forward/back input.
   * @param y Left/right input.
   * @param rotation Rotation input.
   * @return Input stream producing field-relative speeds.
   */
  public SwerveInputStream createInputStream(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
    return SwerveInputStream.of(m_drive, x, y).withControllerRotationAxis(rotation);
  }

  /**
   * Drives field-relative from driver input, with acceleration limiting and heading hold. Use this
   * as the default command.
   *
   * <p>Acceleration is limited first, then heading hold replaces rotation when the driver isn't
   * turning, so the hold's corrections aren't slowed by the limiter. Commands that control rotation
   * themselves (lock, auto-align) must require this subsystem so they interrupt this one.
   *
   * @param speeds Field-relative speeds, e.g. from {@link #createInputStream}.
   * @param fragile While true, use the gentler fragile-piece acceleration limit.
   * @return Command that drives until interrupted.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> speeds, BooleanSupplier fragile) {
    ChassisAccelLimiter limiter =
        new ChassisAccelLimiter(
            () ->
                fragile.getAsBoolean()
                    ? DriveConstants.kFragileLinearAccel
                    : DriveConstants.kMaxLinearAccel,
            () -> DriveConstants.kMaxLinearDecel,
            () -> DriveConstants.kMaxAngularAccel);

    return runOnce(
            () -> {
              limiter.reset(m_drive.getFieldVelocity());
              m_headingHold.reset();
            })
        .andThen(
            run(
                () ->
                    m_drive.driveFieldOriented(
                        m_headingHold.calculate(limiter.calculate(speeds.get())))))
        .withName("Drive");
  }

  /**
   * Zeroes the gyro so the robot's current heading becomes forward.
   *
   * @return Command that zeroes the gyro.
   */
  public Command zeroGyro() {
    return runOnce(m_drive::zeroGyro).withName("Zero Gyro");
  }

  /**
   * Points the wheels in an X so the robot resists being pushed. Safe to press while driving: the
   * robot first brakes at the normal slow-down limit, and only locks once it has nearly stopped.
   * Snapping the wheels to an X at speed would stop the robot almost instantly, scrubbing the
   * treads, loading the modules sideways, and risking a tip.
   *
   * @return Command that brakes, then holds the wheels locked until interrupted.
   */
  public Command lock() {
    ChassisAccelLimiter brake =
        new ChassisAccelLimiter(
            () -> DriveConstants.kMaxLinearDecel,
            () -> DriveConstants.kMaxLinearDecel,
            () -> DriveConstants.kMaxAngularAccel);

    Command slowDown =
        runOnce(() -> brake.reset(m_drive.getFieldVelocity()))
            .andThen(
                run(() -> m_drive.driveFieldOriented(brake.calculate(new ChassisSpeeds())))
                    .until(this::isNearlyStopped))
            // Backstop in case the robot is being pushed and never reads "stopped".
            .withTimeout(DriveConstants.kLockBrakeTimeoutSeconds);

    return slowDown.andThen(run(m_drive::lockPose)).withName("Lock");
  }

  /** True when the robot is moving slowly enough to lock the wheels safely. */
  private boolean isNearlyStopped() {
    ChassisSpeeds speeds = m_drive.getRobotVelocity();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            < DriveConstants.kLockMaxLinearSpeedMps
        && Math.abs(speeds.omegaRadiansPerSecond) < DriveConstants.kLockMaxAngularSpeedRadPerSec;
  }

  /** Returns the robot's pose as estimated by odometry. */
  public Pose2d getPose() {
    return m_drive.getPose();
  }

  /** Resets odometry to the given field-relative, blue-origin pose. */
  public void resetOdometry(Pose2d pose) {
    m_drive.resetOdometry(pose);
  }

  /** Returns the robot's heading as reported by the gyro. */
  public Rotation2d getHeading() {
    return m_drive.getGyro().getRotation3d().toRotation2d();
  }

  /**
   * Adds a vision pose measurement to the pose estimator.
   *
   * @param visionPose Field-relative, blue-origin pose from vision.
   * @param timestampSeconds When the measurement was taken, in FPGA time.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    m_drive.addVisionMeasurement(visionPose, timestampSeconds);
  }
}
