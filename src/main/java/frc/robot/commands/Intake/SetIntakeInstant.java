package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Intake;
import frc.robot.util.Telemetry;

public class SetIntakeInstant extends InstantCommand {
    private final Intake m_intake;
    private double m_speed;

    public SetIntakeInstant(Intake intake, double speed) {
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
        if(m_speed > 0.0) {
            Telemetry.putString("Intake Arrow", "--->");
        } else if (m_speed < 0.0) {
            Telemetry.putString("Intake Arrow", "<---");
        } else {
            Telemetry.putString("Intake Arrow", "O");
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
    
}
