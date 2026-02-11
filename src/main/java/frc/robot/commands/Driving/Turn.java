package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DriveSubsystem;

public class Turn extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final PIDController thetaController;

    private double degToMove;
    private double targetRadians;

    public Turn(DriveSubsystem driveSubsystem, double degrees) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(m_driveSubsystem);

        degToMove = degrees;

        thetaController = new PIDController(3.0, 0.0, 0.1);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(Math.toRadians(2));
    }

    @Override
    public void initialize() {
        double currentRadians = m_driveSubsystem.getHeading().getRadians();
        targetRadians = currentRadians + Math.toRadians(degToMove);

        thetaController.reset();
        thetaController.setSetpoint(targetRadians);
    }

    @Override
    public void execute() {
        double currentRadians = m_driveSubsystem.getHeading().getRadians();

        double omega = thetaController.calculate(currentRadians);

        // Optional clamp for safety
        omega = MathUtil.clamp(omega, -2.0, 2.0);

        m_driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, omega));
    }

    @Override
    public boolean isFinished() {
        return thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
