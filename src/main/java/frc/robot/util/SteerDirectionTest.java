// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;

/**
 * Checks that positive steer motor output turns the Through Bore encoder the positive direction.
 * If a module is backwards the steer PID is positive feedback and the wheel spins or twitches.
 * Pulses each steer motor briefly and flips the motor inversion for any module that is backwards.
 * Robot must be on blocks.
 */
public class SteerDirectionTest {
  private static final double kPulseDuty = 0.08;
  private static final double kPulseSec = 0.3;
  private static final double kMinMoveRot = 0.003;

  private final SwerveModule[] m_modules;
  private final double[] m_start;

  public SteerDirectionTest(SwerveDrive drive) {
    m_modules = drive.getModules();
    m_start = new double[m_modules.length];
  }

  public Command command(Subsystem requirement) {
    return Commands.sequence(
            Commands.runOnce(this::start, requirement),
            Commands.waitSeconds(kPulseSec),
            Commands.runOnce(this::finish))
        .finallyDo(
            () -> {
              for (SwerveModule m : m_modules) {
                m.getAzimuthMotorController().setDutyCycle(0);
              }
            })
        .withName("Steer Direction Test");
  }

  private void start() {
    SmartDashboard.putString("Zeroing/DirectionTest", "running");
    for (int i = 0; i < m_modules.length; i++) {
      var steer = m_modules[i].getAzimuthMotorController();
      m_start[i] = m_modules[i].getRawAbsoluteEncoderAngle().in(Rotations);
      steer.setDutyCycle(kPulseDuty);
    }
  }

  private void finish() {
    StringBuilder summary = new StringBuilder();
    for (int i = 0; i < m_modules.length; i++) {
      var module = m_modules[i];
      var steer = module.getAzimuthMotorController();
      steer.setDutyCycle(0);
      double delta = module.getRawAbsoluteEncoderAngle().in(Rotations) - m_start[i];
      delta -= Math.rint(delta);
      String result;
      if (delta > kMinMoveRot) {
        result = "OK";
      } else if (delta < -kMinMoveRot) {
        boolean inverted = steer.getConfig().getMotorInverted().orElse(false);
        steer.applyConfig(steer.getConfig().withMotorInverted(!inverted));
        result = "BACKWARDS -> flipped inversion to " + !inverted;
      } else {
        result = "DID NOT MOVE (check motor/CAN/cable)";
      }
      String line = module.getName() + " moved " + String.format("%.4f", delta) + " rot: " + result;
      SmartDashboard.putString("Zeroing/DirectionTest/" + module.getName(), line);
      summary.append(line).append("; ");
    }
    System.out.println("[steer-test] " + summary);
    SmartDashboard.putString("Zeroing/DirectionTest", summary.toString());
  }
}
