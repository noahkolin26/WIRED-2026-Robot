package frc.robot.commands.Agitators;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Agitators;

public class RunAgitators extends Command {
    private final Agitators m_agitators;
    private DoubleSupplier m_speed;

    public RunAgitators(Agitators agitators, DoubleSupplier speed) {
        m_agitators = agitators;
        addRequirements(m_agitators);
        
        m_speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_agitators.setAgitators(m_speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_agitators.setAgitators(0.0);
    }
}
