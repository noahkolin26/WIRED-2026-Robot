// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class GeneralizedMecanumDrive extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final DriveSubsystem m_driveSubsystem;
  private DoubleSupplier m_xSpeed;
  private DoubleSupplier m_ySpeed;
  private DoubleSupplier m_rot;
  private BooleanSupplier m_isFieldRelative;

  private final SlewRateLimiter translationSlewRateLimiter = new SlewRateLimiter(0.7);
  private final SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(0.7);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GeneralizedMecanumDrive(DriveSubsystem subsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, BooleanSupplier isFieldRelative) {
    m_driveSubsystem = subsystem;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_rot = rot;
    m_isFieldRelative = isFieldRelative;

    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double limitedXSpeed = translationSlewRateLimiter.calculate(m_xSpeed.getAsDouble());
    double limitedYSpeed = translationSlewRateLimiter.calculate(m_ySpeed.getAsDouble());
    double limitedRot = rotationSlewRateLimiter.calculate(m_rot.getAsDouble());
    

    if(m_isFieldRelative.getAsBoolean()) {
        m_driveSubsystem.driveFieldRelative(limitedXSpeed*5, limitedYSpeed*5, limitedRot*5);
    } else {
        m_driveSubsystem.mecanumDrive(limitedXSpeed*5, limitedYSpeed*5, limitedRot*5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.mecanumDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
