// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Properties;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;

/**
 * Keeps the absolute-encoder zero that the REV Hardware Client stores in each steer Spark MAX.
 *
 * <p>YAGSL/YAMS configure the steer Spark MAXes with "reset safe parameters", which wipes that zero
 * on every boot. So: read the stored zero before the swerve drive is built, then write it back
 * (persisted) once the drive exists. A copy is also kept on the roboRIO, used only if a Spark MAX
 * reads 0 (e.g. the code was killed between the wipe and the restore).
 *
 * <p>The module JSONs must use {@code absoluteEncoderOffset: 0} so the zero is not applied twice.
 */
public final class SparkZeroOffsets {
  /** Steer Spark MAX CAN IDs by YAMS module name. */
  private static final Map<String, Integer> kSteerIds =
      Map.of("frontleft", 3, "frontright", 5, "backleft", 7, "backright", 9);

  private static final File kBackup =
      new File(Filesystem.getOperatingDirectory(), "swerve_zero_offsets.properties");

  private final Map<String, Double> m_zeros = new HashMap<>();

  /** Reads the zero stored in each steer Spark MAX. Call BEFORE building the swerve drive. */
  public SparkZeroOffsets() {
    Properties backup = new Properties();
    try (FileInputStream in = new FileInputStream(kBackup)) {
      backup.load(in);
    } catch (IOException e) {
      // No backup yet.
    }

    for (var entry : kSteerIds.entrySet()) {
      double zero;
      try (SparkMax spark = new SparkMax(entry.getValue(), MotorType.kBrushless)) {
        zero = spark.configAccessor.absoluteEncoder.getZeroOffset();
      }
      if (zero == 0.0) {
        zero = Double.parseDouble(backup.getProperty(entry.getKey(), "0"));
      }
      m_zeros.put(entry.getKey(), zero);
      backup.setProperty(entry.getKey(), Double.toString(zero));
      System.out.println("[zero] " + entry.getKey() + " stored zero = " + zero + " rot");
      if (zero == 0.0) {
        DriverStation.reportWarning(
            "No absolute-encoder zero stored for " + entry.getKey() + ": zero it in REV Client.",
            false);
      }
    }

    try (FileOutputStream out = new FileOutputStream(kBackup)) {
      backup.store(out, "Steer absolute encoder zeros (rotations)");
    } catch (IOException e) {
      DriverStation.reportWarning("Could not save zero backup: " + e.getMessage(), false);
    }
  }

  /**
   * Hands the stored zeros to YAMS. The steer Spark MAXes use the Through Bore encoder directly as
   * feedback, and YAMS writes its configured zero offset (the JSON value, 0) to the Spark MAX on
   * every configure. Putting the REV zero into YAMS's own config keeps it from ever being
   * overwritten. Call AFTER building the drive.
   */
  public void restore(SwerveDrive drive) {
    for (SwerveModule module : drive.getModules()) {
      Double zero = m_zeros.get(module.getName());
      if (zero == null || zero == 0.0) {
        continue;
      }
      var steer = module.getAzimuthMotorController();
      steer.applyConfig(steer.getConfig().withExternalEncoderZeroOffset(Rotations.of(zero)));
    }
  }
}
