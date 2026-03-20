package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Shooter;

public class ChangeShooterIndex extends InstantCommand {
    private final Shooter m_shooter;
    private boolean m_increment;

    public ChangeShooterIndex(Shooter shooter, boolean increment) {
        m_shooter = shooter;
        addRequirements(m_shooter);
        
        m_increment = increment;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_increment) {
            m_shooter.advanceIndex();
        } else {
            m_shooter.reverseIndex();
        }
        m_shooter.updatePowerFromIndex();
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
