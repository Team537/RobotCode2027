// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.motorcontrollers.SmartMotorController;

/**
 * Dashboard-triggered auto-tuner for the swerve drive and steer motors. ROBOT MUST BE ON BLOCKS.
 *
 * <p>It never assumes the gain units. Everything is measured through the same motor-controller
 * path the robot drives with:
 *
 * <ul>
 *   <li><b>Drive kS/kV:</b> ramps open-loop voltage, fits volts = kS + kV * speed, then corrects kV
 *       with feedforward-only speed steps until the wheel reaches the target speed.
 *   <li><b>Drive kP:</b> doubles kP until a speed step overshoots or rings, then uses half of it
 *       (Ziegler-Nichols style).
 *   <li><b>Steer kP/kD:</b> the same search with 60 degree steps. kD comes from the ringing
 *       period.
 * </ul>
 *
 * <p>Results are written to the {@link SwerveModuleTuner} dashboard values and printed. They are
 * not saved to disk: copy them into deploy/swerve/base/modules/pidfproperties.json. If a run fails
 * or is interrupted (including by disabling), the previous gains are restored.
 */
public class SwerveAutoTuner {
  /** Dashboard checkbox the operator must tick before any tune will move the wheels. */
  public static final String kBlocksKey = "Tuning/AutoTune/Robot is on blocks";

  private static final String kStatusKey = "Tuning/AutoTune/Status";
  private static final String kResultsKey = "Tuning/AutoTune/Results";

  private static final double kDriveTargetMps = 1.5;
  private static final double kDriveAbortMps = 4.0;
  private static final double kRampStepVolts = 0.5;
  private static final double kRampMaxVolts = 4.0;
  private static final double kSteerStepDeg = 60.0;
  private static final double kDriveKpStart = 0.001;
  private static final double kSteerKpStart = 0.0005;
  private static final int kMaxTrials = 24;

  private final SwerveDrive m_drive;
  private final SwerveModuleTuner m_tuner;
  private final Subsystem m_requirement;

  /**
   * Creates the auto-tuner.
   *
   * @param drive The drivetrain whose modules are tuned.
   * @param tuner Dashboard tuner that holds, applies and restores the gains.
   * @param requirement Subsystem the tune commands require, so driving stops during a tune.
   */
  public SwerveAutoTuner(SwerveDrive drive, SwerveModuleTuner tuner, Subsystem requirement) {
    m_drive = drive;
    m_tuner = tuner;
    m_requirement = requirement;
    SmartDashboard.putBoolean(kBlocksKey, false);
    SmartDashboard.putString(kStatusKey, "Idle");
  }

  /** Tunes drive kS, kV and kP. */
  public Command tuneDrive() {
    return new TuneCommand("Auto-Tune Drive", true, false);
  }

  /** Tunes steer kP and kD. */
  public Command tuneSteer() {
    return new TuneCommand("Auto-Tune Steer", false, true);
  }

  /** Tunes drive, then steer. */
  public Command tuneAll() {
    return new TuneCommand("Auto-Tune All", true, true);
  }

  // ---------------------------------------------------------------- motor helpers

  private void forEachDrive(Consumer<SmartMotorController> action) {
    for (SwerveModule module : m_drive.getModules()) {
      action.accept(module.getDriveMotorController());
    }
  }

  private void forEachSteer(Consumer<SmartMotorController> action) {
    for (SwerveModule module : m_drive.getModules()) {
      action.accept(module.getAzimuthMotorController());
    }
  }

  private double avgDriveMps() {
    double sum = 0;
    SwerveModule[] modules = m_drive.getModules();
    for (SwerveModule module : modules) {
      sum += module.getDriveMotorController().getMeasurementVelocity().in(MetersPerSecond);
    }
    return sum / modules.length;
  }

  private void setDriveGains(double kp, double ki, double kd, double ks, double kv) {
    forEachDrive(
        motor -> {
          motor.setKp(kp);
          motor.setKi(ki);
          motor.setKd(kd);
          motor.setKs(ks);
          motor.setKv(kv);
          motor.setKa(0);
        });
  }

  private void stopAll() {
    forEachDrive(motor -> motor.setVoltage(Volts.of(0)));
    forEachSteer(motor -> motor.setDutyCycle(0));
  }

  private static void status(String text) {
    SmartDashboard.putString(kStatusKey, text);
    System.out.println("[AutoTune] " + text);
  }

  // ---------------------------------------------------------------- step machinery

  private interface Step {
    default void start() {}

    /** Runs every loop. Returns true when the step is finished. */
    boolean run(double seconds);

    default void end() {}
  }

  private static Step instant(Runnable action) {
    return seconds -> {
      action.run();
      return true;
    };
  }

  private static Step timed(
      double duration, Runnable onStart, DoubleConsumer onRun, BooleanSupplier abort) {
    return new Step() {
      @Override
      public void start() {
        onStart.run();
      }

      @Override
      public boolean run(double seconds) {
        onRun.accept(seconds);
        return seconds >= duration || abort.getAsBoolean();
      }
    };
  }

  private static Step timed(double duration, DoubleConsumer onRun) {
    return timed(duration, () -> {}, onRun, () -> false);
  }

  /** Result of looking at one step response. */
  private record Response(boolean unstable, double periodSeconds, double overshoot) {}

  /**
   * Looks for overshoot or ringing in a step response.
   *
   * @param samples Pairs of {seconds, value}.
   * @param target Final value the step was aiming for.
   */
  private static Response analyze(List<double[]> samples, double target) {
    double band = 0.1 * Math.abs(target);
    double max = Double.NEGATIVE_INFINITY;
    int state = 0;
    List<Double> crossings = new ArrayList<>();
    for (double[] sample : samples) {
      if (sample[0] < 0.05) {
        continue;
      }
      max = Math.max(max, sample[1]);
      int now = sample[1] > target + band ? 1 : sample[1] < target - band ? -1 : 0;
      if (now != 0 && state != 0 && now != state) {
        crossings.add(sample[0]);
      }
      if (now != 0) {
        state = now;
      }
    }
    double overshoot = Double.isInfinite(max) ? 0 : (max - target) / Math.abs(target);
    double period = 0;
    if (crossings.size() >= 3) {
      period = 2.0 * (crossings.get(crossings.size() - 1) - crossings.get(0))
          / (crossings.size() - 1);
    }
    return new Response(crossings.size() >= 3 || overshoot > 0.4, period, overshoot);
  }

  // ---------------------------------------------------------------- the command

  /** Everything learned so far during one run. */
  private static final class Results {
    double kS;
    double kV;
    double driveKp;
    double steerKp;
    double steerKd;
    boolean drive;
    boolean steer;
    final List<double[]> rampPoints = new ArrayList<>();
    double[] steerStart;
  }

  private final class TuneCommand extends Command {
    private final boolean m_doDrive;
    private final boolean m_doSteer;
    private final ArrayDeque<Step> m_steps = new ArrayDeque<>();
    private final Timer m_timer = new Timer();
    private Results m_results = new Results();
    private Step m_current;
    private String m_failure;

    TuneCommand(String name, boolean doDrive, boolean doSteer) {
      m_doDrive = doDrive;
      m_doSteer = doSteer;
      setName(name);
      addRequirements(m_requirement);
    }

    @Override
    public void initialize() {
      m_steps.clear();
      m_current = null;
      m_failure = null;
      m_results = new Results();
      if (!SmartDashboard.getBoolean(kBlocksKey, false)) {
        fail("Refused: tick '" + kBlocksKey + "' only after lifting the robot on blocks.");
        return;
      }
      if (m_doDrive) {
        planDrive();
      }
      if (m_doSteer) {
        planSteer();
      }
    }

    @Override
    public void execute() {
      while (m_failure == null) {
        if (m_current == null) {
          m_current = m_steps.pollFirst();
          if (m_current == null) {
            return;
          }
          m_current.start();
          m_timer.restart();
        }
        if (!m_current.run(m_timer.get())) {
          return;
        }
        m_current.end();
        m_current = null;
      }
    }

    @Override
    public boolean isFinished() {
      return m_failure != null || (m_current == null && m_steps.isEmpty());
    }

    @Override
    public void end(boolean interrupted) {
      stopAll();
      if (m_failure != null || interrupted) {
        m_tuner.reapplyAll();
        status(
            m_failure != null
                ? "FAILED, old gains restored: " + m_failure
                : "Interrupted, old gains restored.");
        return;
      }
      StringBuilder out = new StringBuilder();
      if (m_results.drive) {
        m_tuner.set(SwerveModuleTuner.kDriveKs, m_results.kS);
        m_tuner.set(SwerveModuleTuner.kDriveKv, m_results.kV);
        m_tuner.set(SwerveModuleTuner.kDriveKp, m_results.driveKp);
        out.append(
            String.format(
                "drive kS=%.4f kV=%.4f kP=%.4f; ", m_results.kS, m_results.kV, m_results.driveKp));
      }
      if (m_results.steer) {
        m_tuner.set(SwerveModuleTuner.kSteerKp, m_results.steerKp);
        m_tuner.set(SwerveModuleTuner.kSteerKd, m_results.steerKd);
        out.append(
            String.format("steer kP=%.5f kD=%.5f", m_results.steerKp, m_results.steerKd));
      }
      SmartDashboard.putString(kResultsKey, out.toString());
      status("Done. " + out + " (copy into pidfproperties.json)");
    }

    private void fail(String reason) {
      m_failure = reason;
      m_steps.clear();
    }

    private void add(Step step) {
      m_steps.addLast(step);
    }

    private void addNext(Step... steps) {
      for (int i = steps.length - 1; i >= 0; i--) {
        m_steps.addFirst(steps[i]);
      }
    }

    // ------------------------------------------------------------ drive plan

    private void planDrive() {
      Results r = m_results;
      add(
          instant(
              () -> {
                setDriveGains(0, 0, 0, 0, 0);
                status("Drive: measuring kS/kV with a voltage ramp");
              }));

      for (double v = kRampStepVolts; v <= kRampMaxVolts + 1e-9; v += kRampStepVolts) {
        final double volts = v;
        final double[] acc = new double[2];
        add(
            timed(
                0.8,
                () -> {
                  acc[0] = 0;
                  acc[1] = 0;
                },
                seconds -> {
                  forEachDrive(motor -> motor.setVoltage(Volts.of(volts)));
                  if (seconds > 0.5) {
                    acc[0] += Math.abs(avgDriveMps());
                    acc[1]++;
                  }
                },
                () -> Math.abs(avgDriveMps()) > kDriveAbortMps));
        add(instant(() -> {
          if (acc[1] > 0) {
            r.rampPoints.add(new double[] {volts, acc[0] / acc[1]});
          }
        }));
      }
      add(timed(1.0, seconds -> forEachDrive(motor -> motor.setVoltage(Volts.of(0)))));
      add(instant(this::fitFeedforward));

      // Feedforward-only speed steps; correct kV until the wheel hits the target.
      for (int pass = 0; pass < 2; pass++) {
        final double[] acc = new double[2];
        add(
            timed(
                1.4,
                () -> {
                  acc[0] = 0;
                  acc[1] = 0;
                  status("Drive: checking kV at " + kDriveTargetMps + " m/s");
                },
                seconds -> {
                  forEachDrive(
                      motor -> motor.setVelocity(MetersPerSecond.of(kDriveTargetMps)));
                  if (seconds > 0.8) {
                    acc[0] += avgDriveMps();
                    acc[1]++;
                  }
                },
                () -> Math.abs(avgDriveMps()) > kDriveAbortMps));
        add(
            instant(
                () -> {
                  double reached = acc[1] > 0 ? acc[0] / acc[1] : 0;
                  if (reached < 0.05) {
                    fail("Drive wheel did not follow the speed command.");
                    return;
                  }
                  r.kV *= kDriveTargetMps / reached;
                  setDriveGains(0, 0, 0, r.kS, r.kV);
                }));
        add(timed(0.8, seconds -> forEachDrive(motor -> motor.setVoltage(Volts.of(0)))));
      }

      add(instant(() -> driveTrial(kDriveKpStart, 0)));
    }

    private void fitFeedforward() {
      Results r = m_results;
      double n = 0;
      double sx = 0;
      double sy = 0;
      double sxx = 0;
      double sxy = 0;
      for (double[] point : r.rampPoints) {
        if (point[1] < 0.05) {
          continue;
        }
        n++;
        sx += point[1];
        sy += point[0];
        sxx += point[1] * point[1];
        sxy += point[1] * point[0];
      }
      double denom = n * sxx - sx * sx;
      if (n < 3 || Math.abs(denom) < 1e-9) {
        fail("Drive wheels barely spun during the voltage ramp. Check CAN ids and power.");
        return;
      }
      double slope = (n * sxy - sx * sy) / denom;
      if (slope <= 0) {
        fail("Drive speed did not rise with voltage; check the encoder and motor direction.");
        return;
      }
      r.kV = slope;
      r.kS = Math.max(0, (sy - slope * sx) / n);
      setDriveGains(0, 0, 0, r.kS, r.kV);
      status(String.format("Drive: kS=%.4f kV=%.4f, searching kP", r.kS, r.kV));
    }

    private void driveTrial(double kp, int trial) {
      Results r = m_results;
      List<double[]> samples = new ArrayList<>();
      addNext(
          instant(
              () -> {
                forEachDrive(motor -> motor.setKp(kp));
                samples.clear();
                status(String.format("Drive: trying kP=%.4f", kp));
              }),
          timed(
              1.0,
              () -> {},
              seconds -> {
                forEachDrive(motor -> motor.setVelocity(MetersPerSecond.of(kDriveTargetMps)));
                samples.add(new double[] {seconds, avgDriveMps()});
              },
              () -> Math.abs(avgDriveMps()) > kDriveAbortMps),
          timed(0.8, seconds -> forEachDrive(motor -> motor.setVoltage(Volts.of(0)))),
          instant(
              () -> {
                Response response = analyze(samples, kDriveTargetMps);
                if (response.unstable()) {
                  r.driveKp = 0.5 * kp;
                  r.drive = true;
                  forEachDrive(motor -> motor.setKp(r.driveKp));
                } else if (trial + 1 >= kMaxTrials) {
                  fail("Drive never rang up to kP=" + kp + "; the gain is not taking effect.");
                } else {
                  driveTrial(kp * 2, trial + 1);
                }
              }));
    }

    // ------------------------------------------------------------ steer plan

    private void planSteer() {
      Results r = m_results;
      add(
          instant(
              () -> {
                SwerveModule[] modules = m_drive.getModules();
                r.steerStart = new double[modules.length];
                for (int i = 0; i < modules.length; i++) {
                  r.steerStart[i] =
                      modules[i].getAzimuthMotorController().getMechanismPosition().in(Degrees);
                }
                forEachSteer(
                    motor -> {
                      motor.setKp(0);
                      motor.setKi(0);
                      motor.setKd(0);
                    });
                status("Drive done; steer: searching kP");
              }));
      add(instant(() -> steerTrial(kSteerKpStart, 0)));
    }

    private void commandSteer(double offsetDeg) {
      SwerveModule[] modules = m_drive.getModules();
      for (int i = 0; i < modules.length; i++) {
        modules[i]
            .getAzimuthMotorController()
            .setPosition(Degrees.of(m_results.steerStart[i] + offsetDeg));
      }
    }

    private double avgSteerOffsetDeg() {
      SwerveModule[] modules = m_drive.getModules();
      double sum = 0;
      for (int i = 0; i < modules.length; i++) {
        double now = modules[i].getAzimuthMotorController().getMechanismPosition().in(Degrees);
        sum += MathUtil.inputModulus(now - m_results.steerStart[i], -180, 180);
      }
      return sum / modules.length;
    }

    private void steerTrial(double kp, int trial) {
      Results r = m_results;
      List<double[]> samples = new ArrayList<>();
      addNext(
          instant(
              () -> {
                forEachSteer(motor -> motor.setKp(kp));
                samples.clear();
                status(String.format("Steer: trying kP=%.5f", kp));
              }),
          timed(0.6, seconds -> commandSteer(0)),
          timed(
              1.2,
              seconds -> {
                commandSteer(kSteerStepDeg);
                samples.add(new double[] {seconds, avgSteerOffsetDeg()});
              }),
          timed(0.8, seconds -> commandSteer(0)),
          instant(
              () -> {
                Response response = analyze(samples, kSteerStepDeg);
                if (response.unstable()) {
                  r.steerKp = 0.5 * kp;
                  r.steerKd = response.periodSeconds() > 0 ? r.steerKp * response.periodSeconds() / 8 : 0;
                  r.steer = true;
                  forEachSteer(
                      motor -> {
                        motor.setKp(r.steerKp);
                        motor.setKd(r.steerKd);
                      });
                } else if (trial + 1 >= kMaxTrials) {
                  fail("Steer never rang up to kP=" + kp + "; the gain is not taking effect.");
                } else {
                  steerTrial(kp * 2, trial + 1);
                }
              }));
    }
  }
}
