package frc.robot.subsystems;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightVision extends SubsystemBase {

    private DriveSubsystem drive;
    private AprilTagFieldLayout fieldLayout;
    private static final String LIMELIGHT_NAME = "limelight"; 
    // Change if your Limelight name is different in the web UI

    public LimelightVision() {
        fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    }

    public void setDrive(DriveSubsystem drive) {
        this.drive = drive;
    }

    /**
     * Gets the robot pose estimated by AprilTags.
     */
    public Pose2d getEstimatedPose() {
        LimelightHelpers.PoseEstimate estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

        if (estimate != null && estimate.tagCount > 0) {
            return estimate.pose;
        }

        return null;
    }

    public boolean hasTargets() {
        return LimelightHelpers.getTV("limelight");
    }

    public boolean seesTag(int tagID) {
        if (!LimelightHelpers.getTV("limelight")) return false;
        return LimelightHelpers.getFiducialID("limelight") == tagID;
    }

    public double getTX() {
        return LimelightHelpers.getTX("limelight");
    }

    public Optional<Double> getDistanceToTag(int tagID) {

        Optional<Pose2d> tagPose =
            fieldLayout.getTagPose(tagID).map(p -> p.toPose2d());

        if (tagPose.isEmpty()) {
            return Optional.empty();
        }

        Pose2d robotPose = drive.getPose();

        double dx = tagPose.get().getX() - robotPose.getX();
        double dy = tagPose.get().getY() - robotPose.getY();

        double distance = Math.hypot(dx, dy);

        return Optional.of(distance);
    }
}