package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;
import yams.mechanisms.swerve.SwerveDrive;

class StickDriftTest {
  private static final int kLeftX = 0;
  private static final int kLeftY = 1;
  private static final int kRightX = 4;

  @Test
  void restingRotationStickDoesNotSteerWheelsIntoAnX() throws Exception {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setJoystickAxisCount(0, 6);
    DriverStationSim.setJoystickButtonCount(0, 10);

    RobotContainer container = new RobotContainer();
    SwerveDrive drive = (SwerveDrive) field(field(container, "m_swerve"), "m_drive");

    runLoops(150, -0.8, -0.5, -0.6);
    runLoops(200, 0.0, 0.0, 0.12);

    SwerveModuleState[] commanded = drive.getDesiredModuleStates();
    SwerveModuleState[] actual = drive.getModuleStates();
    for (int i = 0; i < commanded.length; i++) {
      assertEquals(0.0, commanded[i].speedMetersPerSecond, 1e-6, "module " + i + " speed");
      assertEquals(
          actual[i].angle.getDegrees(), commanded[i].angle.getDegrees(), 1.0,
          "module " + i + " should hold where it is, not swing to an X");
    }
  }

  private static void runLoops(int loops, double leftY, double leftX, double rightX) {
    for (int i = 0; i < loops; i++) {
      DriverStationSim.setJoystickAxis(0, kLeftY, leftY);
      DriverStationSim.setJoystickAxis(0, kLeftX, leftX);
      DriverStationSim.setJoystickAxis(0, kRightX, rightX);
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
      CommandScheduler.getInstance().run();
      SimHooks.stepTiming(0.02);
    }
  }

  private static Object field(Object o, String name) throws Exception {
    Field f = o.getClass().getDeclaredField(name);
    f.setAccessible(true);
    return f.get(o);
  }
}
