// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera("OV9281");
  private final PhotonPoseEstimator poseEstimator;
  private final DriveSubsystem drive;
    
  public Vision(DriveSubsystem drive) {
    poseEstimator = new PhotonPoseEstimator(
      VisionConstants.kAprilTagLayout,
      VisionConstants.kRobotToCamera
    );

    this.drive = drive;
  }

public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var results = camera.getAllUnreadResults();

    Optional<EstimatedRobotPose> latestEstimate = Optional.empty();

    for (var result : results) {
        if (!result.hasTargets()) continue;

        poseEstimator.setReferencePose(drive.getPose());
        latestEstimate = poseEstimator.update(result);
    }

    return latestEstimate;
}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    @Override
  public void simulationPeriodic() {
    
    }

}