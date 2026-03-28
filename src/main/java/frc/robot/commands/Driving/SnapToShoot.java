package frc.robot.commands.Driving;

import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.List;

public class SnapToShoot extends Command {
    private final DriveSubsystem drive;
    private final boolean isRedAlliance;
    private Command pathCommand = Commands.none(); // safe default

    public SnapToShoot(DriveSubsystem drive, boolean isRed) {
        this.drive = drive;
        this.isRedAlliance = isRed;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.getPose();

        /*
        // PathPlanner needs valid (non-zero) Rotation2d objects.
        // new Rotation2d(0) gives cos=1, sin=0 — always valid.
        Rotation2d startRot = currentPose.getRotation();
        System.out.println("Current rotation: " + startRot.getDegrees());
        Rotation2d endRot = targetPose.getRotation();
        System.out.println("Target rotation: " + endRot.getDegrees());

        List<PathPoint> points = List.of(
            new PathPoint(currentPose.getTranslation(), new RotationTarget(0.0, startRot)),
            new PathPoint(targetPose.getTranslation(), new RotationTarget(1.0, endRot))
        );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 6.28, 6.28);

        PathPlannerPath path = PathPlannerPath.fromPathPoints(
            points,
            constraints,
            new GoalEndState(0.0, targetPose.getRotation())
        );

        path.preventFlipping = true;

        pathCommand = AutoBuilder.followPath(path);
        pathCommand.initialize(); // must be called before execute()
        */
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
        drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    /*
    public Pose2d getFirstPoint() {
        
    }

    public Pose2d getSecondPoint() {

    }
    */
}