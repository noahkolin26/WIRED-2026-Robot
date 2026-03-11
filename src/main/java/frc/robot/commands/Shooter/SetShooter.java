package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;

public class SetShooter extends Command {
  private final Shooter m_shooter;
  private double m_power;
  private boolean m_undo = false;

  /**
   * Sets the shooter to a given power.
   *
   * @param shooter The shooter subsystem used by this command.
   * @param power The power to set the shooter to.
   * @param undo If "true", two identical setShooter commands lead to power = 0.
   */
  public SetShooter(Shooter shooter, double power, boolean undo) {
    m_shooter = shooter;
    m_power = power;
    m_undo = undo;
    
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     double power = m_power;

    if (m_undo && m_shooter.getPower() == m_power) {
      //m_power = 0.0; //can't directly set m_power or else it stays zero forver.
      power = 0.0;
    }

    m_shooter.setShooter(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
