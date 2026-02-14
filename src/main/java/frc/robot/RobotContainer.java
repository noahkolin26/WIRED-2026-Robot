// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Driving.*;
import frc.robot.commands.Intake.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  //private final Vision m_vision = new Vision();
  private final Intake m_intake = new Intake();
  // private final Shooter m_shooter = new Shooter();
  // private final Agitators m_agitators = new Agitators();

  private final CommandXboxController xboxController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private boolean fieldRelative = false;

  public RobotContainer() {
    configureBindings();

    m_driveSubsystem.setDefaultCommand(
      new GeneralizedMecanumDrive(
        m_driveSubsystem,
        () -> -getDriverLeftY()*0.5,
        () -> -getDriverLeftX()*0.5,
        () -> -getDriverRightX()*0.5,
        () -> isFieldRelative()
      )
    );

    // new RunCommand(() -> {
    //   m_vision.getEstimatedGlobalPose().ifPresent(estimate -> {
    //     m_driveSubsystem.addVisionMeasurement(
    //         estimate.estimatedPose.toPose2d(),
    //         estimate.timestampSeconds,
    //         VisionConstants.kVisionStdDevs
    //     );
    //   });
    // });

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
    xboxController.b().whileTrue(new MecanumDrive(m_driveSubsystem, constantOn, constantOff, constantOff));
    xboxController.a().whileTrue(new MecanumDrive(m_driveSubsystem, constantOff, constantOn, constantOff));
    xboxController.x().whileTrue(new MecanumDrive(m_driveSubsystem, constantOff, constantOff, constantOn));
    xboxController.leftBumper().whileTrue(new RunIntake(m_intake, () -> Constants.IntakeConstants.defaultReverseIntakePower));
    xboxController.rightBumper().whileTrue(new RunIntake(m_intake, () -> Constants.IntakeConstants.defaultIntakePower));

    // xboxController.y().onTrue(alignToAprilTag(9)); // red side hub; make it dynamic which april tag can be aligned to later
    // xboxController.leftBumper().onTrue(Commands.runOnce(() -> fieldRelative = !fieldRelative));
    xboxController.y()
      .and(xboxController.rightBumper())
      .onTrue(m_driveSubsystem.resetPoseCommand());
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

  // The "driver" specification can be used to build in another controller for mechanisms later if desired.
  public CommandXboxController getXboxController() {
    return xboxController;
  }

  public CommandXboxController getDriverXboxController() {
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

  public boolean isFieldRelative() {
    return fieldRelative;
  }

  public double getRightTrigger() {
    return xboxController.getRightTriggerAxis();
  }
}
