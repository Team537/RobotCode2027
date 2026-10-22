// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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

  public static class VisionConstants {
    /**
     * AprilTag layout. Must match the layout selected in PhotonVision's settings, and the event's
     * field (Welded or AndyMark). A mismatch is the most common vision failure.
     */
    public static final AprilTagFields kFieldLayout = AprilTagFields.k2026RebuiltWelded;

    // Camera names must match the names set in the PhotonVision web UI.
    public static final String kFrontLeftName = "front-left";
    public static final String kFrontRightName = "front-right";

    // Robot center (on the floor) to camera. +x forward, +y left, +z up. Negative pitch tilts the
    // camera up; positive yaw turns it left. PLACEHOLDERS: measure on the robot.
    public static final Transform3d kRobotToFrontLeft =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(8), Units.inchesToMeters(8), Units.inchesToMeters(8)),
            new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(20)));
    public static final Transform3d kRobotToFrontRight =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(8), Units.inchesToMeters(-8), Units.inchesToMeters(8)),
            new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(-20)));

    // How much to trust a vision pose: standard deviations of x (m), y (m) and heading (rad).
    // Larger = trust less. Scaled up with distance to the tags. Tune from measured noise.
    public static final double[] kSingleTagStdDevs = {4, 4, 8};
    public static final double[] kMultiTagStdDevs = {0.5, 0.5, 1};

    // Rejection filters.
    /** Single-tag poses beyond this distance are too noisy to use. */
    public static final double kMaxSingleTagDistanceMeters = 4.0;
    /** PhotonVision's guidance: single-tag ambiguity above 0.2 is likely a flipped solution. */
    public static final double kMaxSingleTagAmbiguity = 0.2;
    /** Poses this far outside the field edge are rejected. */
    public static final double kFieldBorderMarginMeters = 0.5;
    /** Poses this far above or below the floor are rejected. */
    public static final double kMaxHeightErrorMeters = 0.3;
  }
}
