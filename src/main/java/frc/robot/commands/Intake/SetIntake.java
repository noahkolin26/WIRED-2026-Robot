package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;

public class SetIntake extends Command {
    private final Intake m_intake;
    private double m_speed;

    public SetIntake(Intake intake, double speed) {
        m_intake = intake;
        addRequirements(m_intake);
        
        m_speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intake.setIntake(m_speed);
    }

    @Override
    public void end(boolean interrupted) {

    }
}