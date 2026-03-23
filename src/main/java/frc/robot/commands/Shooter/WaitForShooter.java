package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;

public class WaitForShooter extends Command {
    private final Shooter m_shooter;
    private double m_rps;

    public WaitForShooter(Shooter shooter, double rps) {
        m_shooter = shooter;
        addRequirements(m_shooter);
        
        m_rps = rps;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return (m_shooter.getCurrentRPS() > m_rps);
    }
}
