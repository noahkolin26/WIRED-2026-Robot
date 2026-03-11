package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Throat;

public class FullSendIt extends Command {
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Throat m_throat;
    private double m_speed;

    public FullSendIt(Intake intake, Shooter shooter, Throat throat, double speed) {
        m_intake = intake;
        m_shooter = shooter;
        m_throat = throat;
        addRequirements(m_intake, m_shooter, m_throat);
        
        m_speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_shooter.setShooter(m_speed);
        m_throat.setThroatPower(1);
        m_intake.setIntake(1);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntake();
        m_shooter.stopShooter();
        m_throat.stopThroat();
    }
}