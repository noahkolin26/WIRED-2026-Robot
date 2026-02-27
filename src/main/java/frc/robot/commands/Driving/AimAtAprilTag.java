package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightVision;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AimAtAprilTag extends Command {

    private final DriveSubsystem drive;
    private final LimelightVision vision;
    private final int targetTag;

    private final double kP = 0.02; // tune this

    public AimAtAprilTag(DriveSubsystem drive, LimelightVision vision, int tagID) {
        this.drive = drive;
        this.vision = vision;
        this.targetTag = tagID;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (!vision.seesTag(targetTag)) {
            drive.drive(new ChassisSpeeds(0, 0, 0));
            return;
        }

        double tx = vision.getTX();
        double rotationSpeed = -tx * kP;

        drive.drive(new ChassisSpeeds(0, 0, rotationSpeed));
    }

    @Override
    public boolean isFinished() {
        return vision.seesTag(targetTag) && Math.abs(vision.getTX()) < 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds(0, 0, 0));
    }
}