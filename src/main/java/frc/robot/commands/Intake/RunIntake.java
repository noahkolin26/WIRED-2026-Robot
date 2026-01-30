package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
    private final Intake m_intake;
    private DoubleSupplier m_speed;

    public RunIntake(Intake intake, DoubleSupplier speed) {
        m_intake = intake;
        addRequirements(m_intake);
        
        m_speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intake.setIntake(m_speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setIntake(0.0);
    }
}
