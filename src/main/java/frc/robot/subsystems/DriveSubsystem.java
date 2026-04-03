// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/*
 * commit "early no 2" on 3/22
 * 
 * > Problem: The robot probably doesn't know where it is, and maybe the driver doesn't either.
 * > Suspicion: We can fix this through odometry! Forget about vision for now because I don't happen to have a camera on me.
 * > Testing:
 *    - PathPlanner is trying to reset the simPose but it isn't working because the simPose can't (Couldn't) be reset
 *    - LimelightVision gives the "DistanceToTag" (robotPose versus tagPose) and "StableDistanceSupplier":
 *          the odometry distance is actually based on the vision-odometry movement because it just does the Pythagorean thm on the robot pose and the tag
 *          idk if the vision is updating the robot pose at all so it *should* just be odometry
 *          I think the direct "Vision" version and the measurements it incorporates into "Fused"/"StableDistanceSupplier" are
 *            based on seeing the tag and using that directly, not just on vision measurements in general
 *          But the "StableDistanceSupplier" uses some 8 'taps' (?) in its moving average, so the value it was reporting was
 *            1/8 of the odometry. the odometry is in meters, or it should be based on field measurements I found
 * > GPT:
 *    - The hub is located in the center of the field (thanks)
 *    - Let's not use that
 * > Solution:
 *     - Now the sim pose will also reset so autonomouses look correct on the simulator
 *     - IMPORTANT: (Tested in sim) If the robot runs into the center of the front of the hub and the driver holds both triggers for 1.5 seconds, the
 *        robot pose *should* reset to being slammed into the center of the front of the hub. The gyro didn't look like it needed to be reset
 *        but I put the code in there anyway, as a separate line so it is easy to remove
 *     - So now we can have half decent odometry for aligning to the hub (??) or setting the shooter based on lirp
 *          ^^^^ NOT YET IMPLEMENTED as of this commit (#2)
 *     - getStableOdomDistanceSupplier() or whatever it's called in LimelightVision returns the distance to the relevant tag in meters
 *        (hopefully)
 *  > note: the simulator gui will let you set the alliance station, but the first time you enable it the code doesn't always seem to
 *    catch it, so disable/disconnect and "enable"(teleop) it once or twice and it should fix it
 */

import frc.robot.util.Telemetry;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.sim.SimGyro;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.sim.DriveSim;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;

public class DriveSubsystem extends SubsystemBase {
  private SparkMax frontLeftMotor;
  private SparkMax frontRightMotor;
  private SparkMax backLeftMotor;
  private SparkMax backRightMotor;

  RelativeEncoder frontLeftEnc;
  RelativeEncoder frontRightEnc;
  RelativeEncoder backLeftEnc;
  RelativeEncoder backRightEnc;

  private final MecanumDriveKinematics kinematics;
  private final MecanumDriveOdometry odometry;

  private final AHRS gyro;
  private final SimGyro simGyro = new SimGyro();
  private final Field2d field = new Field2d();

  private final PIDController frontLeftPID = new PIDController(0.5, 0.0, 0.1);
  private final PIDController frontRightPID = new PIDController(0.5, 0.0, 0.1);
  private final PIDController backLeftPID = new PIDController(0.5, 0.0, 0.1);
  private final PIDController backRightPID = new PIDController(0.5, 0.0, 0.1);

  private LimelightVision limelightVision;

  private MecanumDrivePoseEstimator poseEstimator;

  private DriveSim sim; // only constructed in simulation
    
  public DriveSubsystem(LimelightVision vision) {
    limelightVision = vision;

    // FRONT LEFT
    SparkMaxConfig frontLeftConfig = new SparkMaxConfig();
    frontLeftConfig.encoder
      .positionConversionFactor(DriveConstants.metersPerRotation)
      .velocityConversionFactor(DriveConstants.metersPerRotation / 60.0);
    frontLeftConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    
    frontLeftMotor = new SparkMax(DriveConstants.kFrontLeftDrivePort, MotorType.kBrushless);
    frontLeftMotor.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontLeftEnc = frontLeftMotor.getEncoder();

    // FRONT RIGHT
    SparkMaxConfig frontRightConfig = new SparkMaxConfig();
    frontRightConfig.encoder
      .positionConversionFactor(DriveConstants.metersPerRotation)
      .velocityConversionFactor(DriveConstants.metersPerRotation / 60.0);
    frontRightConfig
      .idleMode(IdleMode.kBrake)
      .inverted(true);
    
    frontRightMotor = new SparkMax(DriveConstants.kFrontRightDrivePort, MotorType.kBrushless);
    frontRightMotor.configure(frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRightEnc = frontRightMotor.getEncoder();

    // BACK LEFT
    SparkMaxConfig backLeftConfig = new SparkMaxConfig();
    backLeftConfig.encoder
      .positionConversionFactor(DriveConstants.metersPerRotation)
      .velocityConversionFactor(DriveConstants.metersPerRotation / 60.0);
    backLeftConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    
    backLeftMotor = new SparkMax(DriveConstants.kBackLeftDrivePort, MotorType.kBrushless);
    backLeftMotor.configure(backLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backLeftEnc = backLeftMotor.getEncoder();

    // BACK RIGHT
    SparkMaxConfig backRightConfig = new SparkMaxConfig();
    backRightConfig.encoder
      .positionConversionFactor(DriveConstants.metersPerRotation)
      .velocityConversionFactor(DriveConstants.metersPerRotation / 60.0);
    backRightConfig
      .idleMode(IdleMode.kBrake)
      .inverted(true);
    
    backRightMotor = new SparkMax(DriveConstants.kBackRightDrivePort, MotorType.kBrushless);
    backRightMotor.configure(backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backRightEnc = backRightMotor.getEncoder();

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DriverStation.reportError(
          "PathPlanner RobotConfig failed to load. Did you deploy GUI settings?",
          e.getStackTrace()
      );
      throw new RuntimeException(e);
    }

    // wheel distances from center of robot (meters)
    kinematics = DriveConstants.kMecanumDriveKinematics;

    gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);
    gyro.reset();

    odometry = new MecanumDriveOdometry(kinematics, getHeading(), getWheelPositions());

    poseEstimator =
    new MecanumDrivePoseEstimator(
        DriveConstants.kMecanumDriveKinematics,
        getHeading(),
        getWheelPositions(),
        new Pose2d()
    );

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    if (RobotBase.isSimulation()) {
      sim = new DriveSim(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, simGyro);
    }
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
  }

  public void mecanumDrive(double x, double y, double r) {
    ChassisSpeeds speeds = new ChassisSpeeds(x, y, r);

    MecanumDriveWheelSpeeds wheelSpeeds =
        kinematics.toWheelSpeeds(speeds);

    wheelSpeeds.desaturate(DriveConstants.MAX_WHEEL_SPEED);

    frontLeftMotor.set(clamp(wheelSpeeds.frontLeftMetersPerSecond / DriveConstants.MAX_WHEEL_SPEED, -1.0, 1.0));
    frontRightMotor.set(clamp(wheelSpeeds.frontRightMetersPerSecond / DriveConstants.MAX_WHEEL_SPEED, -1.0, 1.0));
    backLeftMotor.set(clamp(wheelSpeeds.rearLeftMetersPerSecond / DriveConstants.MAX_WHEEL_SPEED, -1.0, 1.0));
    backRightMotor.set(clamp(wheelSpeeds.rearRightMetersPerSecond / DriveConstants.MAX_WHEEL_SPEED, -1.0, 1.0));
  }

  double clamp(double value, double min, double max) {
      return Math.max(min, Math.min(max, value));
  }

  public void resetEncoders() {
    frontLeftEnc.setPosition(0);
    frontRightEnc.setPosition(0);
    backLeftEnc.setPosition(0);
    backRightEnc.setPosition(0);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public Command resetGyroCommand() {
    return run(() -> zeroHeading()).withTimeout(0.25);
  }

  public Pose2d getPose() {
    if(RobotBase.isSimulation()) {
      return sim.getPose();
    }

    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        getHeading(),
        getWheelPositions(),
        pose
    );
    // next line new, trying to do auton tests etc in sim - shouldn't break real autos
    if(RobotBase.isSimulation()) {
      sim.resetPose(pose);
    }
    //sim.resetPose(pose);
    poseEstimator.resetPose(pose);
    frontLeftPID.reset();
    frontRightPID.reset();
    backLeftPID.reset();
    backRightPID.reset();
  }

  // found these values in sim (GPT screwed up lol), + and - are for 1/2 robot length (should only be x-axis)
  public Pose2d getHubFrontPose(boolean isRed) {
    if(isRed) {
      return new Pose2d(12.94, 4.02, new Rotation2d(3.14159)); // x + 0.4275
    } else {
      return new Pose2d(3.50, 4.02, new Rotation2d(0)); // x - 0.4275
    }
  }

  public void resetToCenterOfHub(boolean isRed, boolean resetPose) {
      gyro.reset();
      if(RobotBase.isSimulation()) {
        simGyro.reset();
      }
      if(resetPose) {
        resetPose(getHubFrontPose(isRed));
      }
    }

  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(
        new MecanumDriveWheelSpeeds(
            frontLeftEnc.getVelocity(),
            frontRightEnc.getVelocity(),
            backLeftEnc.getVelocity(),
            backRightEnc.getVelocity()
        )
    );
  }

  public void drive(ChassisSpeeds speeds) {
    mecanumDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  StructPublisher<ChassisSpeeds> publisher2 = NetworkTableInstance.getDefault().getStructTopic("MyChassisSpeeds", ChassisSpeeds.struct).publish();

  public void driveFieldRelative(
    double xSpeed,
    double ySpeed,
    double rotSpeed
  ) {
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            rotSpeed,
            getHeading()
        );

    publisher2.set(fieldRelativeSpeeds);

    mecanumDrive(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond, fieldRelativeSpeeds.omegaRadiansPerSecond);
  }

  public Rotation2d getHeading() {
    if(RobotBase.isSimulation()) {
      return simGyro.getRotation2d();
    }

    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  private MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(
        frontLeftEnc.getPosition(),
        frontRightEnc.getPosition(),
        backLeftEnc.getPosition(),
        backRightEnc.getPosition()
    );
  }

  @Override
public void periodic() {
  odometry.update(
      getHeading(),
      getWheelPositions()
  );

  poseEstimator.update(getHeading(), getWheelPositions());

  PoseEstimate est = limelightVision.getPoseEstimate();

  boolean acceptedVision = false;

  if (est != null) {
    Telemetry.putDouble("LL Tag Count", est.tagCount);
    Telemetry.putDouble("LL Avg Tag Dist", est.avgTagDist);
    Telemetry.putPose("LL Raw Pose", est.pose);

    Pose2d currentPose = poseEstimator.getEstimatedPosition();
    double visionDeltaMeters =
        currentPose.getTranslation().getDistance(est.pose.getTranslation());

    Telemetry.putDouble("LL Pose Delta", visionDeltaMeters);

    boolean goodTagCount = est.tagCount >= 2;
    boolean goodSingleTag = est.tagCount == 1 && est.avgTagDist < 2.5;
    boolean reasonableJump = visionDeltaMeters < 1.5;

    if ((goodTagCount || goodSingleTag) && reasonableJump) {
      poseEstimator.addVisionMeasurement(
          est.pose,
          est.timestampSeconds,
          VisionConstants.kVisionStdDevs
      );
      acceptedVision = true;
    }
  }

  Telemetry.putBoolean("LL Vision Accepted", acceptedVision);

  if (RobotBase.isSimulation() && sim != null) {
    field.setRobotPose(sim.getPose());
    Telemetry.putDouble("simPoseX", getPose().getX());
    Telemetry.putDouble("simPoseY", getPose().getY());
    Telemetry.putDouble("simPoseRot", getPose().getRotation().getRotations());
  }

  if (RobotBase.isReal()) {
    Telemetry.putPose("Robot Pose", getPose());
    Telemetry.putFieldPose("MainField", getPose());
  }
}


  @Override
  public void simulationPeriodic() {
    if (sim != null) {
      sim.update();
    }

    Telemetry.putPose("Sim Robot Pose", sim.getPose());
    Telemetry.putFieldPose("MainField", sim.getPose());
  }

  public Pose2d getSimPose() {
    return (sim != null) ? sim.getPose() : new Pose2d();
  }
}
