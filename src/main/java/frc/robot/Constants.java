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
    public static final double kDriveDeadband = 0.05;
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
  }
}
