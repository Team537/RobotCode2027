// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/**
 * Holds the robot's heading when the driver lets go of the rotation stick, so bumps and uneven
 * wheel wear don't slowly turn the robot.
 *
 * <p>After the stick is released, it waits briefly for the robot to stop turning, then locks onto
 * the heading at that moment and uses a PID loop to stay there.
 */
public class HeadingHold {
  /** Rotation requests smaller than this (rad/s) count as "stick released". */
  private static final double kRotationDeadband = 0.05;

  /** Time to let the robot stop turning before locking the heading. */
  private static final double kLatchDelaySeconds = 0.2;

  // Starting gains from the AdvantageKit swerve template. Tune on the robot.
  private final ProfiledPIDController m_pid =
      new ProfiledPIDController(5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));

  private final Supplier<Rotation2d> m_heading;
  private final Timer m_sinceRelease = new Timer();
  private Rotation2d m_target = null;

  /**
   * Creates a heading hold.
   *
   * @param heading The robot's heading. Use the pose estimate's rotation, not raw gyro yaw, so it
   *     stays correct after an odometry reset.
   */
  public HeadingHold(Supplier<Rotation2d> heading) {
    m_heading = heading;
    m_pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Forgets the held heading. Call this when the drive command starts. */
  public void reset() {
    m_target = null;
    m_pid.reset(m_heading.get().getRadians());
    m_sinceRelease.restart();
  }

  /**
   * Passes driver rotation through unchanged, or replaces it with a correction toward the held
   * heading when the driver isn't rotating.
   *
   * @param desired Field-relative speeds from the driver.
   * @return Speeds with rotation replaced by the hold correction when holding.
   */
  public ChassisSpeeds calculate(ChassisSpeeds desired) {
    if (Math.abs(desired.omegaRadiansPerSecond) > kRotationDeadband) {
      // Driver is rotating.
      m_target = null;
      m_sinceRelease.restart();
      return desired;
    }
    if (m_target == null) {
      if (!m_sinceRelease.hasElapsed(kLatchDelaySeconds)) {
        return desired;
      }
      m_target = m_heading.get();
      m_pid.reset(m_target.getRadians());
    }
    double omega = m_pid.calculate(m_heading.get().getRadians(), m_target.getRadians());
    return new ChassisSpeeds(desired.vxMetersPerSecond, desired.vyMetersPerSecond, omega);
  }
}
