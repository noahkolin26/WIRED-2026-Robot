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

public class TranslateRobot extends Command {
    private final DriveSubsystem drive;
    private double distance;
    private boolean vertical;
    private Command pathCommand = Commands.none(); // safe default

    public TranslateRobot(DriveSubsystem drive, double distance, boolean vertical) {
        this.drive = drive;
        this.distance = distance;
        this.vertical = vertical;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.getPose();
        Translation2d transform = new Translation2d(vertical ? 0 : distance, vertical ? distance : 0);

        List<PathPoint> points = List.of(
            new PathPoint(currentPose.getTranslation(), new RotationTarget(0.0, currentPose.getRotation())),
            new PathPoint(currentPose.getTranslation().plus(transform), new RotationTarget(1.0, currentPose.getRotation()))
        );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 6.28, 6.28);

        PathPlannerPath path = PathPlannerPath.fromPathPoints(points, constraints, new GoalEndState(0.0, currentPose.getRotation()));

        path.preventFlipping = true;

        pathCommand = AutoBuilder.followPath(path);
        pathCommand.initialize(); // must be called before execute()
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
}