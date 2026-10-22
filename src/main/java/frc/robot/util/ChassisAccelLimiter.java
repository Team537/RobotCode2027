// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;

/**
 * Limits how fast commanded chassis speeds can change so the robot accelerates smoothly.
 *
 * <p>Translation is limited as a vector, so a diagonal move keeps its direction and the limit is
 * the same at every heading. Two separate x/y slew limiters would distort diagonals. Rotation is
 * limited on its own. Limits are suppliers so they can change at runtime, for example when holding
 * a fragile game piece.
 */
public class ChassisAccelLimiter {
  private static final double kLoopSeconds = 0.02;

  private final DoubleSupplier m_maxLinearAccel;
  private final DoubleSupplier m_maxLinearDecel;
  private final DoubleSupplier m_maxAngularAccel;

  private Translation2d m_prevLinear = Translation2d.kZero;
  private double m_prevOmega = 0.0;

  /**
   * Creates a limiter.
   *
   * @param maxLinearAccel Max speed-up rate in m/s².
   * @param maxLinearDecel Max slow-down rate in m/s². Keep this at or above the accel limit so the
   *     driver can always stop.
   * @param maxAngularAccel Max change in rotation rate in rad/s².
   */
  public ChassisAccelLimiter(
      DoubleSupplier maxLinearAccel, DoubleSupplier maxLinearDecel, DoubleSupplier maxAngularAccel) {
    m_maxLinearAccel = maxLinearAccel;
    m_maxLinearDecel = maxLinearDecel;
    m_maxAngularAccel = maxAngularAccel;
  }

  /**
   * Starts the limiter from the robot's current speeds, so it doesn't fight a robot that's already
   * moving. Call this when the drive command starts.
   *
   * @param current The robot's current speeds, in the same frame as later calls to {@link
   *     #calculate}.
   */
  public void reset(ChassisSpeeds current) {
    m_prevLinear = new Translation2d(current.vxMetersPerSecond, current.vyMetersPerSecond);
    m_prevOmega = current.omegaRadiansPerSecond;
  }

  /**
   * Moves one loop's worth from the last output toward the desired speeds.
   *
   * @param desired Speeds the driver is asking for.
   * @return Speeds to command this loop.
   */
  public ChassisSpeeds calculate(ChassisSpeeds desired) {
    Translation2d want = new Translation2d(desired.vxMetersPerSecond, desired.vyMetersPerSecond);
    Translation2d delta = want.minus(m_prevLinear);

    // Slowing down gets the decel limit; speeding up or changing direction gets the accel limit.
    boolean slowing = want.getNorm() < m_prevLinear.getNorm();
    double maxStep = (slowing ? m_maxLinearDecel : m_maxLinearAccel).getAsDouble() * kLoopSeconds;
    if (delta.getNorm() > maxStep) {
      delta = delta.times(maxStep / delta.getNorm());
    }
    m_prevLinear = m_prevLinear.plus(delta);

    double omegaStep = m_maxAngularAccel.getAsDouble() * kLoopSeconds;
    double omegaDelta = desired.omegaRadiansPerSecond - m_prevOmega;
    m_prevOmega += Math.copySign(Math.min(Math.abs(omegaDelta), omegaStep), omegaDelta);

    return new ChassisSpeeds(m_prevLinear.getX(), m_prevLinear.getY(), m_prevOmega);
  }
}
