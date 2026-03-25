package frc.robot.commands.Shooter;

import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class DynamicSetShooter extends Command {
  private final Shooter m_shooter;
  private final DoubleSupplier m_distance;

  /**
   * Runs the shooter based on linear interpolation of distance to target. The distance is provided by a DoubleSupplier, which can be a lambda that gets the distance from a vision subsystem or other sensor.
   * The power is calculated using the getShootingPower method, which converts distance to inches and uses a LinearInterpolator to get the appropriate power from a predefined curve.
   * 
   * @param shooter The shooter subsystem used by this command.
   * @param distance The distance supplier used by this command.
   */
  public DynamicSetShooter(Shooter shooter, DoubleSupplier distance) {
    m_shooter = shooter;
    m_distance = distance;
    
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = m_distance.getAsDouble();
    double power = getShootingPower(distance);
    m_shooter.setShooter(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.resetIndex();
  }

  // given distance from the target in meters, return the estimated power
  public double getShootingPower(double distance) {
    double inches = distance * 39.3701; // convert meters to inches
    
    return ShooterConstants.shootPowerLirp.getInterpolatedValue(inches);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
