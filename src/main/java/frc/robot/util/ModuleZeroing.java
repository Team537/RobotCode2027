// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;

/**
 * Publishes each steer module's absolute encoder reading. The zero itself is set in the REV
 * Hardware Client and kept by {@link SparkZeroOffsets}; the Through Bore encoder is the steer
 * feedback directly, so there is nothing to seed.
 */
public class ModuleZeroing {
  private final SwerveModule[] m_modules;

  public ModuleZeroing(SwerveDrive drive) {
    m_modules = drive.getModules();
  }

  /** Call every loop. */
  public void periodic() {
    for (SwerveModule module : m_modules) {
      String name = "Zeroing/" + module.getName() + "/";
      double rot = module.getRawAbsoluteEncoderAngle().in(Degrees) / 360.0;
      // Zero-adjusted Through Bore position, as the REV Client shows it: 0 (or 1) = straight.
      SmartDashboard.putNumber(name + "AdjustedRotations", rot);
      SmartDashboard.putNumber(name + "AbsoluteRawDeg", rot * 360.0);
      SmartDashboard.putNumber(
          name + "SteerMotorDeg",
          module.getAzimuthMotorController().getMechanismPosition().in(Degrees));
    }
  }
}
