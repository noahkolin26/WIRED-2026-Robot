package frc.robot.commands.Throat;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Throat;

public class SetThroatInstant extends InstantCommand {
    private final Throat m_throat;
    private double m_speed;

    public SetThroatInstant(Throat throat, double speed) {
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
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
