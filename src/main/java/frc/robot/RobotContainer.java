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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

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

  public static boolean isRedAlliance = false;

  private final CommandXboxController xboxController1 =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController xboxController2 =
      new CommandXboxController(OperatorConstants.kShooterControllerPort);

  private boolean fieldRelative = false;

  public RobotContainer() {
    m_limelightVision.setDrive(m_driveSubsystem);

    configureBindings();

    m_driveSubsystem.setDefaultCommand(
      new GeneralizedMecanumDrive(
        m_driveSubsystem,
        () -> -getDriverLeftY(),//*0.5,
        () -> -getDriverLeftX(),//*0.5,
        () -> -getDriverRightX(),//*0.5,
        () -> isFieldRelative()
      )
    );
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

    //Driver Controller
    xboxController1.leftBumper().onTrue(Commands.runOnce(() -> fieldRelative = !fieldRelative));
    xboxController1.rightBumper().whileTrue(new VisionHeadingAssist(m_driveSubsystem, m_limelightVision, hubAprilTag(), () -> -getDriverLeftY(), () -> -getDriverLeftX()));

    xboxController1.a().onTrue(new SetIntake(m_intake, IntakeConstants.defaultIntakePower));
    xboxController1.b().onTrue(new SetIntake(m_intake, 0));
    //xboxController1.x().onTrue(new SetIntake(m_intake, -0.1));  might need an emergency reverse

    //This isn't working right now.  Even when holding the right trigger it is not moving.
    //xboxController1.rightTrigger(0.1).whileTrue(new SetIntake(m_intake, IntakeConstants.defaultIntakePower));
    //xboxController1.rightTrigger(0.1).onFalse(new SetIntake(m_intake, 0));

    //Shooter controller
    xboxController2.y().onTrue(new SetShooter(m_shooter, ShooterConstants.shootPowerFULL, true));
    xboxController2.b().onTrue(new SetShooter(m_shooter, ShooterConstants.shootPowerLONG, true));
    xboxController2.x().onTrue(new SetShooter(m_shooter, ShooterConstants.shootPowerMEDIUM, true));
    xboxController2.a().onTrue(new SetShooter(m_shooter, ShooterConstants.shootPowerSHORT, true));

    xboxController2.pov(0).whileTrue(new DynamicRunShooter(m_shooter, m_limelightVision.getStableDistanceSupplier(hubAprilTag())));

    xboxController2.rightBumper().onTrue(new SetThroat(m_throat,1.0));
    xboxController2.leftBumper().onTrue(new SetThroat(m_throat,0.0));

    //throat button on controller 2 for testing
    // xboxController2.a().onTrue(new SetThroat(m_throat,1.0));
    // xboxController2.b().onTrue(new SetThroat(m_throat, -1.0));
    // xboxController2.y().onTrue(new SetThroat(m_throat,0.0));
  }

  DoubleSupplier constantOn = () -> 1.0;
  DoubleSupplier constantOff = () -> 0.0;

  public Command alignToAprilTag(int tagID) {
  return Commands.defer(
      () -> {
        Pose2d targetPose =
            DriveSubsystem.getAlignmentPose(
                tagID,
                DriveConstants.kShootFromTag
            );

        if (targetPose == null) {
          return Commands.none();
        }

        return AutoBuilder.pathfindToPose(
            targetPose,
            new PathConstraints(
                2.0,  // max velocity (m/s)
                1.5,   // max accel (m/s^2)
                2.0, // max ang velocity (rad/s)
                2.0  // max ang acceleratoin (rad/s^2)
            ),
            0.0
        );
      },
      Set.of(m_driveSubsystem) // requirements
  );
}



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  // The hub has two AprilTags, this only tracks one of them
  public int hubAprilTag() {
    if(isRedAlliance) {
      return 9;
    } else {
      return 25;
    }
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
