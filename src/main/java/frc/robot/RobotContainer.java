// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Driving.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.Throat.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;

import choreo.auto.AutoChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final LimelightVision m_limelightVision = new LimelightVision();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_limelightVision);
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Throat m_throat = new Throat();

  private final SendableChooser<Command> autoChooser;

  public static boolean isRedAlliance = false;

  private final CommandXboxController xboxController1 =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController xboxController2 =
      new CommandXboxController(OperatorConstants.kShooterControllerPort);

      // this was bothering me too so now the default is false
  public boolean fieldRelative = false;

  public RobotContainer() {
    m_limelightVision.setDrive(m_driveSubsystem);

    configureBindings();

    // Shooting power (off, short, medium, long, full)
    NamedCommands.registerCommand("shooterOff", new SetShooterInstant(m_shooter, 0.0));
    NamedCommands.registerCommand("shooterShort", new SetShooterInstant(m_shooter, ShooterConstants.shootPowerSHORT));
    NamedCommands.registerCommand("shooterMedium", new SetShooterInstant(m_shooter, ShooterConstants.shootPowerMEDIUM));
    NamedCommands.registerCommand("shooterLong", new SetShooterInstant(m_shooter, ShooterConstants.shootPowerLONG));
    NamedCommands.registerCommand("shooterFull", new SetShooterInstant(m_shooter, ShooterConstants.shootPowerFULL));

    // Intake power (off, default, full, reverse)
    NamedCommands.registerCommand("intakeOff", new SetIntakeInstant(m_intake, 0.0));
    NamedCommands.registerCommand("intakeDefault", new SetIntakeInstant(m_intake, IntakeConstants.defaultIntakePower));
    NamedCommands.registerCommand("intakeDefaultReverse", new SetIntakeInstant(m_intake, IntakeConstants.defaultReverseIntakePower));
    NamedCommands.registerCommand("intakeFull", new SetIntakeInstant(m_intake, 1.0));

    // Throat power (off, full, reverse)
    NamedCommands.registerCommand("throatOff", new SetThroatInstant(m_throat, 0.0));
    NamedCommands.registerCommand("throatFull", new SetThroatInstant(m_throat, 1.0));
    NamedCommands.registerCommand("throatReverse", new SetThroatInstant(m_throat, -1.0));
    NamedCommands.registerCommand("throatFullIfShooter", new SetThroatInstant(m_throat, 1.0).onlyIf(() -> m_shooter.getCurrentRPS() > 40));

    m_driveSubsystem.setDefaultCommand(
      new GeneralizedMecanumDrive(
        m_driveSubsystem,
        () -> -getDriverLeftY(),//*0.5,
        () -> -getDriverLeftX(),//*0.5,
        () -> -getDriverRightX(),//*0.5,
        () -> isFieldRelative()
      )
    );

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
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

    // Driver controller part 2
    xboxController1.leftBumper().onTrue(Commands.runOnce(() -> fieldRelative = !fieldRelative));
    xboxController1.rightBumper().whileTrue(new VisionHeadingAssist(m_driveSubsystem, m_limelightVision, hubAprilTag(), () -> -getDriverLeftY(), () -> -getDriverLeftX()));

    Trigger bothDriverTriggers = new Trigger(() -> ((xboxController1.getLeftTriggerAxis() > 0.5) && (xboxController1.getRightTriggerAxis() > 0.5))).debounce(1.5);
    bothDriverTriggers.onTrue(Commands.runOnce(() -> m_driveSubsystem.resetToCenterOfHub(isRedAlliance)));

    xboxController1.b().onTrue(new SetIntake(m_intake, IntakeConstants.defaultIntakePower));
    xboxController1.a().onTrue(new SetIntake(m_intake, 0.0));
    // xboxController1.b().onTrue(Commands.parallel(new SetIntake(m_intake, IntakeConstants.defaultIntakePower), new SetThroat(m_throat, -1.0)));
    // xboxController1.a().onTrue(Commands.parallel(new SetIntake(m_intake, 0.0), new SetThroat(m_throat, 0.0)));

    // Shooter controller part 2
    xboxController2.leftBumper().onTrue(new ChangeShooterIndex(m_shooter, false).withTimeout(0.2));
    xboxController2.rightBumper().onTrue(new ChangeShooterIndex(m_shooter, true).withTimeout(0.2));
    xboxController2.b().onTrue(new SetShooter(m_shooter, 0.0, false));

    xboxController2.leftTrigger().whileTrue(new SetThroat(m_throat, 1.0).onlyIf(() -> m_shooter.getCurrentRPS() > 40));
    xboxController2.leftTrigger().onFalse(new SetThroat(m_throat, -1.0));
    //xboxController2.rightTrigger().whileTrue(new SetThroat(m_throat, -1.0));

    xboxController2.x().whileTrue(new DynamicSetShooter(m_shooter, m_limelightVision.getStableDistanceSupplier(hubAprilTag())));

    //throat button on controller 2 for testing
    // xboxController2.a().onTrue(new SetThroat(m_throat,1.0));
    // xboxController2.b().onTrue(new SetThroat(m_throat, -1.0));
    // xboxController2.y().onTrue(new SetThroat(m_throat,0.0));
  }

  DoubleSupplier constantOn = () -> 1.0;
  DoubleSupplier constantOff = () -> 0.0;

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // The hub has two AprilTags, this only tracks one of them
  public int hubAprilTag() {
    if(isRedAlliance) {
      return 9;
    } else {
      return 25;
    }
  }

  public Command shootWaitIntake(double shootPower, double waitTime, double intakePower) {
    return Commands.sequence(
      new SetShooter(m_shooter, shootPower, false),
      new WaitCommand(waitTime),
      new SetIntake(m_intake, intakePower),
      new SetThroat(m_throat, 1)
    );
  }

  public void turnOffAllMotors() {
    m_driveSubsystem.mecanumDrive(0, 0, 0);
    m_intake.setIntake(0);
    m_shooter.setShooter(0);
    m_throat.setThroatPower(0);
  }

  // The "driver" specification can be used to build in another controller for mechanisms later if desired.
  public CommandXboxController getXboxController() {
    return xboxController1;
  }

  public CommandXboxController getDriverXboxController() {
    return xboxController2;
  }

  public double getDriverLeftY() {
    return xboxController1.getLeftY();
  }

  public double getDriverRightY() {
    return xboxController1.getRightY();
  }

  public double getDriverLeftX() {
    return xboxController1.getLeftX();
  }

  public double getDriverRightX() {
    return xboxController1.getRightX();
  }

  public double getLeftTrigger() {
    return xboxController1.getLeftTriggerAxis();
  }

  public boolean isFieldRelative() {
    return fieldRelative;
  }

  public double getRightTrigger() {
    return xboxController1.getRightTriggerAxis();
  }
}
