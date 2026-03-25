package frc.robot.commands.Driving;

import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.List;

public class DriveToPose extends Command {
    private final DriveSubsystem drive;
    private final Pose2d targetPose;
    private Command pathCommand;

    public DriveToPose(DriveSubsystem drive, Pose2d targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.getPose();

        if (currentPose.getTranslation().equals(targetPose.getTranslation())) {
            // Already at target; just rotate in place if needed
            drive.drive(new ChassisSpeeds(0, 0, 0));
            return;
        }

        Translation2d delta = targetPose.getTranslation().minus(currentPose.getTranslation());
        if (delta.getNorm() < 0.01) {
            // If you’re essentially at the target, skip the path
            drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
            return;
        }

        Rotation2d startRot = currentPose.getRotation();
        Rotation2d endRot = targetPose.getRotation();

        // Guard against (0,0)
        if (startRot.getCos() == 0 && startRot.getSin() == 0) startRot = new Rotation2d(0);
        if (endRot.getCos() == 0 && endRot.getSin() == 0) endRot = new Rotation2d(0);

        // Create PathPoints: start at current pose, end at target pose
        List<PathPoint> points = List.of(
            new PathPoint(
                currentPose.getTranslation(),
                new RotationTarget(0.0, startRot)
            ),
            new PathPoint(
                targetPose.getTranslation(),
                new RotationTarget(1.0, endRot)
            )
        );

        // Global path constraints (max velocity, max acceleration)
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 6.28, 6.28);

        // Create the path
        PathPlannerPath path = PathPlannerPath.fromPathPoints(
            points,
            constraints,
            new GoalEndState(0.0, targetPose.getRotation()) // end speed 0, face target rotation
        );

        path.preventFlipping = true;

        // Follow the path using your AutoBuilder
        pathCommand = AutoBuilder.followPath(path);
    }

    @Override
    public void execute() {
        pathCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        pathCommand.end(interrupted);
        drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0)); // stops drivetrain
    }
}