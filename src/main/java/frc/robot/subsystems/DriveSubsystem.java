// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import frc.robot.Constants.DriveConstants;
import frc.robot.sim.DriveSim;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.DriverStation;

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

  private MecanumDrivePoseEstimator poseEstimator;

  private DriveSim sim; // only constructed in simulation
    
  public DriveSubsystem() {
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
    kinematics =
      new MecanumDriveKinematics(
          new Translation2d(0.3,  0.3),  // Front Left
          new Translation2d(0.3, -0.3),  // Front Right
          new Translation2d(-0.3,  0.3), // Back Left
          new Translation2d(-0.3, -0.3)  // Back Right
      );

    gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);
    gyro.reset();

    odometry = new MecanumDriveOdometry(kinematics, getHeading(), getWheelPositions());

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
  public Command resetEncodersCommand() {
    return run(() -> resetEncoders()).withTimeout(0.25);
  }
  public Pose2d getPose() {
    if(RobotBase.isSimulation()) {
      return sim.getPose();
    }

    return odometry.getPoseMeters();
  }
  public Command resetPoseCommand() {
    return run(() -> resetPose(Pose2d.kZero)).withTimeout(0.25);
  }
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        getHeading(),
        getWheelPositions(),
        pose
    );
    frontLeftPID.reset();
    frontRightPID.reset();
    backLeftPID.reset();
    backRightPID.reset();

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
  private Rotation2d getHeading() {
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
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
        getHeading(),
        getWheelPositions()
    );

    // Display pose if in sim; otherwise you'll want to do real odometry here.
    if (RobotBase.isSimulation() && sim != null) {
      field.setRobotPose(sim.getPose());
    }

    if (RobotBase.isReal()) {
      publisher.set(getPose());
    }
  }
    @Override
  public void simulationPeriodic() {
    if (sim != null) {
      sim.update();
    }

    publisher.set(sim.getPose());
  }
  public Pose2d getSimPose() {
    return (sim != null) ? sim.getPose() : new Pose2d();
  }
  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
  StructPublisher<Rotation2d> rotPublisher = NetworkTableInstance.getDefault().getStructTopic("MyRotation", Rotation2d.struct).publish();
}
