package frc.robot.commands.Driving;

import frc.robot.Constants.AimingConstants;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.List;
import java.util.function.BooleanSupplier;

public class SnapToShoot extends Command {
    private final DriveSubsystem drive;
    private final boolean isRedAlliance;
    private Command pathCommand = Commands.none(); // safe default

    private Pose2d currentPose;
    private Pose2d firstPoint;
    private Pose2d secondPoint;
    private BooleanSupplier booleanSupplier;

    public SnapToShoot(DriveSubsystem drive, boolean isRed, BooleanSupplier booleanSupplier) {
        this.drive = drive;
        this.isRedAlliance = isRed;
        addRequirements(drive);

        currentPose = drive.getPose();
        firstPoint = getFirstPoint(currentPose);
        secondPoint = getSecondPoint(firstPoint);
        this.booleanSupplier = booleanSupplier;
    }

    @Override
    public void initialize() {
        currentPose = drive.getPose();
        
        // PathPlanner needs valid (non-zero) Rotation2d objects.
        // new Rotation2d(0) gives cos=1, sin=0 — always valid.

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 6.28, 6.28);

        pathCommand = AutoBuilder.pathfindToPose(
            secondPoint,
            constraints,
            0.0
        ).until(booleanSupplier);

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
    
    public Pose2d getFirstPoint(Pose2d pose) {
        if(isRedAlliance) {
            return pose.nearest(AimingConstants.redFirstPoints);
        } else {
            return pose.nearest(AimingConstants.blueFirstPoints);
        }
    }

    public Pose2d getSecondPoint(Pose2d firstPose) {
        if(isRedAlliance) {
            return firstPose.nearest(AimingConstants.redSecondPoints);
        } else {
            return firstPose.nearest(AimingConstants.blueSecondPoints);
        }
    }
}