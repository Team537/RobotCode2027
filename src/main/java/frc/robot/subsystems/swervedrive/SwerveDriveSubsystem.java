// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import org.json.simple.parser.ParseException;
import swervelib.parser.SwerveParser;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SwerveDriveTelemetryConfig;

/**
 * Swerve drivetrain. YAGSL builds the hardware from the JSON config in deploy/swerve/base, and
 * YAMS's {@link SwerveDrive} handles kinematics, odometry, simulation and telemetry.
 */
public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveDrive m_drive;

  /** Creates the drivetrain from the YAGSL config in deploy/swerve/base. */
  public SwerveDriveSubsystem() {
    SwerveDriveConfig config =
        new SwerveDriveConfig()
            .withSubsystem(this)
            .withStartingPose(new Pose2d(3, 3, Rotation2d.kZero))
            .withTranslationController(new PIDController(4, 0, 0))
            .withRotationController(new PIDController(1, 0, 0))
            .withTelemetry("swerve", new SwerveDriveTelemetryConfig(TelemetryVerbosity.HIGH));

    SwerveParser.parse(new File(Filesystem.getDeployDirectory(), "swerve/base"));
    m_drive = SwerveParser.createSwerveDrive(config);

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
        m_drive::getRobotRelativeSpeed,
        (speeds, feedforwards) ->
            m_drive.setRobotRelativeChassisSpeeds(speeds, feedforwards.linearForces()),
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
    return new SwerveInputStream(m_drive, x, y, rotation);
  }

  /**
   * Drives field-relative using the speeds from an input stream.
   *
   * @param stream Input stream from {@link #createInputStream}.
   * @return Command that drives until interrupted.
   */
  public Command driveFieldOriented(SwerveInputStream stream) {
    return run(() -> m_drive.setFieldRelativeChassisSpeeds(stream.get())).withName("Drive");
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
   * Points the wheels in an X so the robot resists being pushed.
   *
   * @return Command that holds the wheels locked until interrupted.
   */
  public Command lock() {
    return run(m_drive::lockPose).withName("Lock");
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
    return new Rotation2d(m_drive.getGyroAngle());
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

  @Override
  public void periodic() {
    m_drive.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_drive.simIterate();
  }
}
