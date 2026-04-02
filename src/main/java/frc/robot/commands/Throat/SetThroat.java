package frc.robot.commands.Throat;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Throat;
import frc.robot.util.Telemetry;

public class SetThroat extends Command {
    private final Throat m_throat;
    private double m_speed;

    public SetThroat(Throat throat, double speed) {
        m_throat = throat;
        addRequirements(m_throat);
        
        m_speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_throat.setThroatPower(m_speed);       //.getAsDouble());
        if(m_speed > 0.0) {
            Telemetry.putString("Throat Arrow", "--->");
        } else if (m_speed < 0.0) {
            Telemetry.putString("Throat Arrow", "<---");
        } else {
            Telemetry.putString("Throat Arrow", "O");
        }
    }

    @Override
    public void end(boolean interrupted) {
        // WARNING: IT IS NOW IMPOSSIBLE TO STOP THE THROAT WITHOUT DIRECTLY SETTING IT TO ZERO
    }
}
