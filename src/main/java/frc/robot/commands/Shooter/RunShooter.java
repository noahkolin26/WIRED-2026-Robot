package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
    private final Shooter m_shooter;
    private DoubleSupplier m_speed;

    public RunShooter(Shooter shooter, DoubleSupplier speed) {
        m_shooter = shooter;
        addRequirements(m_shooter);
        
        m_speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooter.setShooter(m_speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setShooter(0.0);
    }
}
