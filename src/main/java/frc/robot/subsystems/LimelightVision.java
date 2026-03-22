package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import frc.robot.util.Telemetry;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.RobotContainer;

import java.util.Optional;
import java.util.function.DoubleSupplier;

public class LimelightVision extends SubsystemBase {

    private DriveSubsystem drive;
    private AprilTagFieldLayout fieldLayout;
    private static final String LIMELIGHT_NAME = "limelight"; 
    // Change if your Limelight name is different in the web UI

    private final NetworkTable table =
        NetworkTableInstance.getDefault().getTable("Vision");

    private final BooleanPublisher seesTagPublisher =
        table.getBooleanTopic("SeesTag").publish();

    private final DoublePublisher distancePublisher =
        table.getDoubleTopic("TagDistance").publish();

    public LimelightVision() {
        fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    }

    public void setDrive(DriveSubsystem drive) {
        this.drive = drive;
    }

    private final LinearFilter txFilter = LinearFilter.movingAverage(5);

    public double getFilteredTX() {
        return txFilter.calculate(LimelightHelpers.getTX(LIMELIGHT_NAME));
    }

    /**
     * Gets the robot pose estimated by AprilTags.
     */
    public Pose2d getEstimatedPose() {
        LimelightHelpers.PoseEstimate estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

        if (estimate != null && estimate.tagCount > 0) {
            Telemetry.putFieldPose("Limelight Pose", estimate.pose);

            return estimate.pose;
        }

        return null;
    }

    public boolean hasTargets() {
        return LimelightHelpers.getTV(LIMELIGHT_NAME);
    }

    public boolean seesTag(int tagID) {
        if (!LimelightHelpers.getTV(LIMELIGHT_NAME)) return false;
        return LimelightHelpers.getFiducialID(LIMELIGHT_NAME) == tagID;
    }

    public double getTX() {
        return LimelightHelpers.getTX(LIMELIGHT_NAME);
    }

    @Override
    public void periodic() {
        int hubAprilTag = RobotContainer.isRedAlliance ? 9 : 25;
        boolean seesTag = seesTag(hubAprilTag);
        seesTagPublisher.set(seesTag);

        distancePublisher.set(
            getStableDistanceSupplier(hubAprilTag).getAsDouble()
        );

        Telemetry.putDouble("? odom distance to tag", getStableOdomDistanceSupplier(hubAprilTag).getAsDouble());
        Telemetry.putDouble("? ideal shoot power", ShooterConstants.shootPowerLirp.getInterpolatedValue(getStableOdomDistanceSupplier(hubAprilTag).getAsDouble()));
    }

    public double idealShootPower() {
        return ShooterConstants.shootPowerLirp.getInterpolatedValue(getStableOdomDistanceSupplier(RobotContainer.isRedAlliance ? 9 : 25).getAsDouble());
    }

    public Optional<Double> getDirectDistanceToTag(int tagID) {
        if (!LimelightHelpers.getTV(LIMELIGHT_NAME)) {
            return Optional.empty();
        }

        if (LimelightHelpers.getFiducialID(LIMELIGHT_NAME) != tagID) {
            return Optional.empty();
        }

        double[] pose = LimelightHelpers.getTargetPose_RobotSpace(LIMELIGHT_NAME);

        if (pose == null || pose.length < 6) {
            return Optional.empty();
        }

        double x = pose[0]; // left/right relative to robot
        double z = pose[2]; // forward distance

        double distance = Math.hypot(x, z);

        if (distance > 10 || distance < 0.1) {
            return Optional.empty();
        }

        return Optional.of(distance);
    }

    public Optional<Double> getFusedDistanceToTag(int tagID) {
        Optional<Double> visionDistance = getDirectDistanceToTag(tagID);
        Optional<Double> odometryDistance = getDistanceToTag(tagID);

        if (visionDistance.isEmpty() && odometryDistance.isEmpty()) {
            return Optional.empty();
        }

        if (visionDistance.isEmpty()) {
            return odometryDistance;
        }

        if (odometryDistance.isEmpty()) {
            return visionDistance;
        }

        double visionWeight = 0.7;
        double odometryWeight = 0.3;

        double fused =
            visionDistance.get() * visionWeight +
            odometryDistance.get() * odometryWeight;

        return Optional.of(fused);
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

    public DoubleSupplier getStableDistanceSupplier(int tagID) {
        MedianFilter spikeFilter = new MedianFilter(5);
        LinearFilter smoothingFilter = LinearFilter.movingAverage(8);

        final double[] lastValue = {0.0};
        final boolean[] hasValue = {false};

        return () -> {

            Optional<Double> distanceOpt = getFusedDistanceToTag(tagID);

            if (distanceOpt.isPresent()) {

                double raw = distanceOpt.get();

                // Remove spikes
                double despiked = spikeFilter.calculate(raw);

                // Smooth normal jitter
                double smoothed = smoothingFilter.calculate(despiked);

                lastValue[0] = smoothed;
                hasValue[0] = true;
            }

            if (hasValue[0]) {
                return lastValue[0];
            }

            return 0.0;
    };
}

    public DoubleSupplier getStableOdomDistanceSupplier(int tagID) {
        MedianFilter spikeFilter = new MedianFilter(5);
        LinearFilter smoothingFilter = LinearFilter.movingAverage(1);

        final double[] lastValue = {0.0};
        final boolean[] hasValue = {false};

        return () -> {

            Optional<Double> distanceOpt = getDistanceToTag(tagID);

            if (distanceOpt.isPresent()) {

                double raw = distanceOpt.get();

                // Remove spikes
                double despiked = spikeFilter.calculate(raw);

                // Smooth normal jitter
                double smoothed = smoothingFilter.calculate(despiked);

                lastValue[0] = smoothed;
                hasValue[0] = true;
            }

            if (hasValue[0]) {
                return lastValue[0];
            }

            return 0.0;
    };
}

}