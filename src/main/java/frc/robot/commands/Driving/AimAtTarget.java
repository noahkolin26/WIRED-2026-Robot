package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;

public class AimAtTarget extends Command {
    private final DriveSubsystem drive;
    private final Pose2d target;
    private final PIDController rotationController = new PIDController(3.0, 0.0, 0.1);

    public AimAtTarget(DriveSubsystem drive, Pose2d target) {
        this.drive = drive;
        this.target = target;

        addRequirements(drive);

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Math.toRadians(2));
    }

    @Override
    public void initialize() {
        rotationController.reset();
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getPose();
        
        Rotation2d desiredHeading = target.getTranslation().minus(robotPose.getTranslation()).getAngle();

        double output = rotationController.calculate(robotPose.getRotation().getRadians(), desiredHeading.getRadians());
        drive.drive(new ChassisSpeeds(0.0, 0.0, output));
    }

    @Override
    public boolean isFinished() {
        return rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
