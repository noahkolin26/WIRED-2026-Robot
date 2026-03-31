// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kShooterControllerPort = 1;
  }

  public static final class DriveConstants {
    public static final int kFrontLeftDrivePort = 1;
    public static final int kFrontRightDrivePort = 2;
    public static final int kBackLeftDrivePort = 3;
    public static final int kBackRightDrivePort = 4;

    // Geometry (meters): distance from robot center to each wheel
    // If your wheelbase is 24" and trackwidth is 24", then half is 12" = 0.3048m
    public static final double kHalfWheelbaseMeters = Units.inchesToMeters(12.0);
    public static final double kHalfTrackwidthMeters = Units.inchesToMeters(12.0);

    // Convert encoder units to meters
    // Wheel + gearing
    public static final double kWheelRadiusMeters = Units.inchesToMeters(3.0); // 6" diameter wheel
    public static final double kWheelDiameterMeters = kWheelRadiusMeters * 2; // 6 inches
    public static final double kDriveGearing = 12.76; // motor rotations per 1 wheel rotation (example)

    // Encoder conversion factors (NEO integrated encoder reports motor rotations/RPM)
    public static final double kMotorRotationsToWheelMeters = (2.0 * Math.PI * kWheelRadiusMeters) / kDriveGearing;
    public static final double kMotorRPMToWheelMps = (kMotorRotationsToWheelMeters / 60.0);

    // Sim tuning (rough but works)
    public static final double kWheelMoi = 0.001; // kg*m^2 (rough guess)
    public static final double kDtSeconds = 0.02;

    public static final double metersPerRotation = Math.PI * kWheelDiameterMeters / kDriveGearing;

    public static final double MAX_WHEEL_SPEED = 4.5; // meters/sec

    public static final MecanumDriveKinematics kMecanumDriveKinematics = new MecanumDriveKinematics(
      new Translation2d(-0.273, 0.273),
      new Translation2d(0.273, 0.273),
      new Translation2d(-0.273, -0.273),
      new Translation2d(0.273, -0.273));

    public static final Transform2d kShootFromTag =
      new Transform2d(
          new Translation2d(-1.2, 0.0),
          Rotation2d.fromDegrees(180)
      );

  }
  
  public static class IntakeConstants {
    public static final int kIntakePort = 5;

    public static final double defaultIntakePower = 0.8;
    public static final double defaultReverseIntakePower = 0.9;
  }

  public static class ThroatConstants {
    public static final int kThroatLeftPort = 8;
    public static final int kThroatRightPort = 9;

    public static final double defaultThroatPower = 1.0;
  }

  public static class ShooterConstants {
    public static final int kShooterPort = 7;

    public static final double defaultShooterPower = 0.5;

    // keep experimental results here so lirp is easy to update - CHANGED to be in METERS
    public static final double[][] distancePowerPairs = {
      {2.921, 0.5},  //{115, 0.5}, // in inches
      {4.445, 0.6},  //{175, 0.6},
      {4.826, 0.7}   //{190, 0.7}
    };
    public static final double shootPowerSHORT = 0.6;
    public static final double shootPowerMEDIUM = 0.7;
    public static final double shootPowerLONG = 0.75;
    public static final double shootPowerFULL = 1;

    public static final LinearInterpolator shootPowerLirp = new LinearInterpolator(distancePowerPairs);
  }

  public static class AimingConstants {
    public static final Pose2d redHubLocation = new Pose2d(11.909, 4.054, new Rotation2d(0));
    public static final Pose2d blueHubLocation = new Pose2d(4.587, 4.054, new Rotation2d(0));

    public static final Pose2d blueLeftShoot = new Pose2d(4, 7.4, new Rotation2d(-Math.PI/2));
    public static final Pose2d blueRightShoot = new Pose2d(4, 0.67, new Rotation2d(Math.PI/2));
    public static final Pose2d blueLeftTrench = new Pose2d(5.16, 7.4, new Rotation2d(-Math.PI/2));
    public static final Pose2d blueRightTrench = new Pose2d(5.16, 0.67, new Rotation2d(Math.PI/2));
    public static final Pose2d redLeftShoot = new Pose2d(12.5, 0.67, new Rotation2d(Math.PI/2));
    public static final Pose2d redRightShoot = new Pose2d(12.5, 7.4, new Rotation2d(-Math.PI/2));
    public static final Pose2d redLeftTrench = new Pose2d(11.37, 0.67, new Rotation2d(Math.PI/2));
    public static final Pose2d redRightTrench = new Pose2d(11.37, 7.4, new Rotation2d(-Math.PI/2));

    public static final ArrayList<Pose2d> redFirstPoints = new ArrayList<>(List.of(redLeftShoot, redRightShoot, redLeftTrench, redRightTrench));
    public static final ArrayList<Pose2d> redSecondPoints = new ArrayList<>(List.of(redLeftShoot, redRightShoot));
    public static final ArrayList<Pose2d> blueFirstPoints = new ArrayList<>(List.of(blueLeftShoot, blueRightShoot, blueLeftTrench, blueRightTrench));
    public static final ArrayList<Pose2d> blueSecondPoints = new ArrayList<>(List.of(blueLeftShoot, blueRightShoot));
  }

  public static class VisionConstants {
    public static final String kCameraName = "limelight";
    public static final Transform3d kRobotToCamera =
      new Transform3d(
        new Translation3d(0.32, -0.1, 0.30), // forward left up in meters from the center of the robot
        new Rotation3d(0.0, 0.0, 0.0)
      );
    
    public static final AprilTagFieldLayout kAprilTagLayout;

    static {
        try {
            Path tagPath = Filesystem.getDeployDirectory()
                    .toPath()
                    .resolve("src\\main\\deploy\\2026-rebuilt-andymark.json");

            kAprilTagLayout = new AprilTagFieldLayout(tagPath);
        } catch (IOException e) {
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }
    }

    // estimated vision error (x=y=0.7m, angle=1rad), tune based on how stable pose looks, could be dynamically adjusted based on number/proximity of tags
    public static final Matrix<N3, N1> kVisionStdDevs = VecBuilder.fill(0.7, 0.7, 1.0);
  }
}
