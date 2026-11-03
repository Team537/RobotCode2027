// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Consumer;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.motorcontrollers.SmartMotorController;

/**
 * Live-tunes the swerve drive and steer PID/feedforward gains from the dashboard.
 *
 * <p>Edit a value under "Tuning/Drive" or "Tuning/Steer" and it is sent to all four modules on the
 * next loop. Only changed values are sent, to keep CAN traffic down. Starting values mirror
 * deploy/swerve/base/modules/pidfproperties.json. Values set here are lost on reboot: copy the
 * good ones back into that file.
 */
public class SwerveModuleTuner {
  private static final String kDrive = "Tuning/Drive/";
  private static final String kSteer = "Tuning/Steer/";

  /** Dashboard keys, shared with {@link SwerveAutoTuner}. */
  public static final String kDriveKp = kDrive + "kP";

  public static final String kDriveKi = kDrive + "kI";
  public static final String kDriveKd = kDrive + "kD";
  public static final String kDriveKs = kDrive + "kS";
  public static final String kDriveKv = kDrive + "kV";
  public static final String kSteerKp = kSteer + "kP";
  public static final String kSteerKi = kSteer + "kI";
  public static final String kSteerKd = kSteer + "kD";

  private final SwerveDrive m_drive;
  private final Gain[] m_gains;

  /** One dashboard value and the setter it feeds. */
  private static final class Gain {
    final String key;
    final Consumer<Double> apply;
    final java.util.function.Function<SwerveModule, SmartMotorController> motor;
    double last;

    Gain(
        String key,
        double initial,
        java.util.function.Function<SwerveModule, SmartMotorController> motor,
        java.util.function.BiConsumer<SmartMotorController, Double> setter,
        SwerveDrive drive) {
      this.key = key;
      this.motor = motor;
      this.last = initial;
      this.apply =
          value -> {
            for (SwerveModule module : drive.getModules()) {
              setter.accept(motor.apply(module), value);
            }
          };
      SmartDashboard.putNumber(key, initial);
    }

    void update() {
      double value = SmartDashboard.getNumber(key, last);
      if (value != last) {
        last = value;
        apply.accept(value);
      }
    }
  }

  public SwerveModuleTuner(SwerveDrive drive) {
    m_drive = drive;
    m_gains =
        new Gain[] {
          drive(kDriveKp, 0.1, SmartMotorController::setKp),
          drive(kDriveKi, 0.0, SmartMotorController::setKi),
          drive(kDriveKd, 0.03, SmartMotorController::setKd),
          drive(kDriveKs, 0.0, SmartMotorController::setKs),
          drive(kDriveKv, 0.0, SmartMotorController::setKv),
          drive(kDrive + "kA", 0.0, SmartMotorController::setKa),
          steer(kSteerKp, 1.5, SmartMotorController::setKp),
          steer(kSteerKi, 0.0, SmartMotorController::setKi),
          steer(kSteerKd, 0.0, SmartMotorController::setKd),
        };
  }

  private Gain drive(
      String key, double initial, java.util.function.BiConsumer<SmartMotorController, Double> set) {
    return new Gain(key, initial, SwerveModule::getDriveMotorController, set, m_drive);
  }

  private Gain steer(
      String key, double initial, java.util.function.BiConsumer<SmartMotorController, Double> set) {
    return new Gain(key, initial, SwerveModule::getAzimuthMotorController, set, m_drive);
  }

  /** Sets a dashboard value; the next {@link #periodic()} sends it to the motors. */
  public void set(String key, double value) {
    SmartDashboard.putNumber(key, value);
  }

  /** Re-sends every stored gain to the motors and resets the dashboard to match. */
  public void reapplyAll() {
    for (Gain gain : m_gains) {
      SmartDashboard.putNumber(gain.key, gain.last);
      gain.apply.accept(gain.last);
    }
  }

  /** Call every loop. Pushes changed gains and publishes setpoint vs. actual for graphing. */
  public void periodic() {
    for (Gain gain : m_gains) {
      gain.update();
    }

    var actual = m_drive.getModuleStates();
    var desired = m_drive.getDesiredModuleStates();
    double[] actualMps = new double[actual.length];
    double[] desiredMps = new double[actual.length];
    double[] actualDeg = new double[actual.length];
    double[] desiredDeg = new double[actual.length];
    for (int i = 0; i < actual.length; i++) {
      actualMps[i] = actual[i].speedMetersPerSecond;
      desiredMps[i] = desired[i].speedMetersPerSecond;
      actualDeg[i] = actual[i].angle.getDegrees();
      desiredDeg[i] = desired[i].angle.getDegrees();
    }
    SmartDashboard.putNumberArray("Tuning/Drive/ActualMps", actualMps);
    SmartDashboard.putNumberArray("Tuning/Drive/DesiredMps", desiredMps);
    var modules = m_drive.getModules();
    double[] duty = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      duty[i] = modules[i].getAzimuthMotorController().getDutyCycle();
    }
    SmartDashboard.putNumberArray("Tuning/Steer/Duty", duty);
    SmartDashboard.putNumberArray("Tuning/Steer/ActualDeg", actualDeg);
    SmartDashboard.putNumberArray("Tuning/Steer/DesiredDeg", desiredDeg);
  }
}
