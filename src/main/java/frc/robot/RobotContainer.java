// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
// import frc.robot.commands.Driving.FieldRelativeMecanumDrive;
import frc.robot.commands.Driving.MecanumDrive;
import frc.robot.subsystems.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Agitators;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Agitators m_agitators = new Agitators();

  private final XboxController xboxController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  public static SlewRateLimiter mecanumSpeedLimiter = new SlewRateLimiter(Constants.kMecanumSlewRate);
  public static SlewRateLimiter mecanumTurnLimiter = new SlewRateLimiter(Constants.kMecanumRotateSlewRate);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

    // currently robot-relative, field-relative commented out below
    m_driveSubsystem.setDefaultCommand(new MecanumDrive(m_driveSubsystem, () -> -mecanumSpeedLimiter.calculate(getDriverLeftX()),
    () -> -mecanumSpeedLimiter.calculate(getDriverLeftY()), () -> -mecanumSpeedLimiter.calculate(getDriverRightX())));

    //m_driveSubsystem.setDefaultCommand(new FieldRelativeMecanumDrive(m_driveSubsystem, () -> mecanumSpeedLimiter.calculate(getDriverLeftX()),
    //() -> mecanumSpeedLimiter.calculate(getDriverLeftY()), () -> mecanumSpeedLimiter.calculate(getDriverRightY())));
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new MecanumDrive(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_driveSubsystem);
  }

  // The "driver" specification can be used to build in another controller for mechanisms later if desired.
  public XboxController getXboxController() {
    return xboxController;
  }

  public XboxController getDriverXboxController() {
    return xboxController;
  }

  public double getDriverLeftY() {
    return xboxController.getLeftY();
  }

  public double getDriverRightY() {
    return xboxController.getRightY();
  }

  public double getDriverLeftX() {
    return xboxController.getLeftX();
  }

  public double getDriverRightX() {
    return xboxController.getRightX();
  }

  public double getLeftTrigger() {
    return xboxController.getLeftTriggerAxis();
  }

  public double getRightTrigger() {
    return xboxController.getRightTriggerAxis();
  }
}
