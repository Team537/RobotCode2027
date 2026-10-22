// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // The robot's subsystems and commands are defined here...
  private final SwerveDriveSubsystem m_swerve = new SwerveDriveSubsystem();

  // Controller axes are negated because pushing a stick forward or left reads negative.
  private final SwerveInputStream m_driveStream =
      m_swerve
          .createInputStream(
              () -> -m_driverController.getLeftY(),
              () -> -m_driverController.getLeftX(),
              () -> -m_driverController.getRightX())
          .withDeadband(OperatorConstants.kDriveDeadband)
          // Cubing makes the first half of stick travel gentle while keeping full speed at full
          // stick. Speed scaling is done in driverSpeeds(), not with the stream's scale methods,
          // because YAMS scales *before* cubing (a 0.3 scale would become 0.3³ ≈ 3%).
          .withCubeTranslationControllerAxis()
          .withCubeRotationControllerAxis()
          .withAllianceRelativeControl();

  // Hold left bumper for slow, precise driving.
  private final Trigger m_precisionMode = m_driverController.leftBumper();

  // Hold right bumper for gentler acceleration, e.g. while carrying a fragile game piece. Replace
  // with a game-piece sensor or superstructure state once one exists.
  private final Trigger m_fragileMode = m_driverController.rightBumper();

  // Lists every auto in deploy/pathplanner/autos.
  private final SendableChooser<Command> m_autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Left stick translates, right stick X rotates.
    m_swerve.setDefaultCommand(m_swerve.driveFieldOriented(this::driverSpeeds, m_fragileMode));

    // Start re-zeroes field-oriented forward to wherever the robot is facing.
    m_driverController.start().onTrue(m_swerve.zeroGyro());

    // Hold X to lock the wheels in an X and resist pushing.
    m_driverController.x().whileTrue(m_swerve.lock());
  }

  /** Driver stick speeds, scaled down for normal rotation and for precision mode. */
  private ChassisSpeeds driverSpeeds() {
    ChassisSpeeds speeds = m_driveStream.get();
    boolean precise = m_precisionMode.getAsBoolean();
    double translationScale = precise ? OperatorConstants.kPrecisionTranslationScale : 1.0;
    double rotationScale =
        precise ? OperatorConstants.kPrecisionRotationScale : OperatorConstants.kRotationScale;
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond * translationScale,
        speeds.vyMetersPerSecond * translationScale,
        speeds.omegaRadiansPerSecond * rotationScale);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
