package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;

public class SetShooter extends Command {
  private final Shooter m_shooter;
  private final double m_power;

  /**
   * Sets the shooter to a given power.
   *
   * @param shooter The shooter subsystem used by this command.
   * @param power The power to set the shooter to.
   */
  public SetShooter(Shooter shooter, double power) {
    m_shooter = shooter;
    m_power = power;
    
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setShooter(m_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
