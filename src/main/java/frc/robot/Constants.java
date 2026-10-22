// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    // Stick values smaller than this are treated as zero so the robot doesn't creep.
    public static final double kDriveDeadband = 0.10;

    // Fractions of max speed at full stick. Applied after input cubing, so full stick gives
    // exactly this fraction.
    /** Normal rotation, so full stick isn't the fastest possible spin. */
    public static final double kRotationScale = 0.6;
    /** Precision mode (hold left bumper). */
    public static final double kPrecisionTranslationScale = 0.3;
    public static final double kPrecisionRotationScale = 0.4;
  }

  public static class DriveConstants {
    /** Kraken X60 free speed (6000 rpm, no FOC) through 4.71:1 onto a 3 in wheel. */
    public static final double kMaxModuleSpeedMps = 5.08;

    /**
     * Top speed the chassis is allowed to request. 85% of free speed is a safe start; replace with
     * the measured plateau from the max-speed test, and keep PathPlanner's maxDriveSpeed in sync.
     */
    public static final double kMaxChassisSpeedMps = 4.3;

    public static final double kMaxAngularSpeedDps = 540.0;

    // Acceleration limits for driver control. Starting points; tune with the driver by raising
    // until wheels slip, the battery browns out, the robot rocks, or the driver is happy.
    /** Speed-up limit, m/s². */
    public static final double kMaxLinearAccel = 3.0;
    /** Slow-down limit, m/s². Keep at or above kMaxLinearAccel so the robot can always stop. */
    public static final double kMaxLinearDecel = 4.0;
    /** Rotation rate change limit, rad/s². */
    public static final double kMaxAngularAccel = 12.0;
    /** Gentler speed-up limit while carrying a fragile game piece (hold right bumper), m/s². */
    public static final double kFragileLinearAccel = 1.5;

    // X-lock only engages below these speeds; above them it brakes first.
    public static final double kLockMaxLinearSpeedMps = 0.3;
    public static final double kLockMaxAngularSpeedRadPerSec = 0.5;
    /**
     * Longest the X-lock will spend braking before locking anyway. Stopping from full speed at
     * kMaxLinearDecel takes about 1.1 s.
     */
    public static final double kLockBrakeTimeoutSeconds = 1.5;
  }
}
