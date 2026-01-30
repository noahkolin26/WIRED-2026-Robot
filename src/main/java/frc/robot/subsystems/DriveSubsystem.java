// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.sim.SimGyro;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.*;

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
import edu.wpi.first.math.MathUtil;

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

  private final PIDController frontLeftPID = new PIDController(1.2, 0.0, 0.1);
  private final PIDController frontRightPID = new PIDController(1.2, 0.0, 0.1);
  private final PIDController backLeftPID = new PIDController(1.2, 0.0, 0.1);
  private final PIDController backRightPID = new PIDController(1.2, 0.0, 0.1);

  private DriveSim sim; // only constructed in simulation
    
  public DriveSubsystem() {
    // FRONT LEFT
    SparkMaxConfig frontLeftConfig = new SparkMaxConfig();
    frontLeftConfig.encoder.positionConversionFactor(DriveConstants.metersPerRotation);
    frontLeftConfig.encoder.velocityConversionFactor(DriveConstants.metersPerRotation / 60.0);
    // config the config
    frontLeftMotor = new SparkMax(DriveConstants.kFrontLeftDrivePort, MotorType.kBrushless);
    frontLeftMotor.configure(frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontLeftEnc = frontLeftMotor.getEncoder();

    // FRONT RIGHT
    SparkMaxConfig frontRightConfig = new SparkMaxConfig();
    frontRightConfig.encoder.positionConversionFactor(DriveConstants.metersPerRotation);
    frontRightConfig.encoder.velocityConversionFactor(DriveConstants.metersPerRotation / 60.0);
    // config the config
    frontRightMotor = new SparkMax(DriveConstants.kFrontRightDrivePort, MotorType.kBrushless);
    frontRightMotor.configure(frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // BACK LEFT
    SparkMaxConfig backLeftConfig = new SparkMaxConfig();
    backLeftConfig.encoder.positionConversionFactor(DriveConstants.metersPerRotation);
    backLeftConfig.encoder.velocityConversionFactor(DriveConstants.metersPerRotation / 60.0);
    // config the config
    backLeftMotor = new SparkMax(DriveConstants.kBackLeftDrivePort, MotorType.kBrushless);
    backLeftMotor.configure(backLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // BACK RIGHT
    SparkMaxConfig backRightConfig = new SparkMaxConfig();
    backRightConfig.encoder.positionConversionFactor(DriveConstants.metersPerRotation);
    backRightConfig.encoder.velocityConversionFactor(DriveConstants.metersPerRotation / 60.0);
    // config the config
    backRightMotor = new SparkMax(DriveConstants.kBackRightDrivePort, MotorType.kBrushless);
    backRightMotor.configure(backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    RobotConfig config;
    config = new RobotConfig(0, 0, null, 0); // here to remove errors
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
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
   * Drives the robot mecanum style using input from two sticks.
   */
  public void mecanumDrive(double x, double y, double r) {
    double[] motorSpeeds = new double[4];
    motorSpeeds[0] = x + y + r;
    motorSpeeds[1] = x - y - r;
    motorSpeeds[2] = x - y + r;
    motorSpeeds[3] = x + y - r;
    double max = Arrays.stream(motorSpeeds).max().getAsDouble();
    if(max > 1) {
      for(int i = 0; i<motorSpeeds.length; i++) {
        motorSpeeds[i] /= max;
      }
    }
    frontLeftMotor.set(motorSpeeds[0]);
    frontRightMotor.set(motorSpeeds[1]);
    backLeftMotor.set(motorSpeeds[2]);
    backRightMotor.set(motorSpeeds[3]);
  }

  /**
   * Resets all encoders to 0 position.
   */
  public void resetEncoders() {
    frontLeftEnc.setPosition(0);
    frontRightEnc.setPosition(0);
    backLeftEnc.setPosition(0);
    backRightEnc.setPosition(0);
  }

  /** Reset the gyro */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the command for resetting the encoders.
   * 
   * @return A Command.
   */
  public Command resetEncodersCommand() {
    return run(() -> resetEncoders()).withTimeout(0.25);
  }

  /** Returns the robot pose on the field */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Resets odometry to a known pose */
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

  /** Returns robot-relative chassis speeds */
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

  /** Drives the robot */
  public void drive(ChassisSpeeds speeds) {

  MecanumDriveWheelSpeeds targetWheelSpeeds =
      kinematics.toWheelSpeeds(speeds);

  // Prevent commanding impossible speeds
  targetWheelSpeeds.desaturate(DriveConstants.MAX_WHEEL_SPEED);

  double flOutput = frontLeftPID.calculate(
      frontLeftEnc.getVelocity(),
      targetWheelSpeeds.frontLeftMetersPerSecond);

  double frOutput = frontRightPID.calculate(
      frontRightEnc.getVelocity(),
      targetWheelSpeeds.frontRightMetersPerSecond);

  double blOutput = backLeftPID.calculate(
      backLeftEnc.getVelocity(),
      targetWheelSpeeds.rearLeftMetersPerSecond);

  double brOutput = backRightPID.calculate(
      backRightEnc.getVelocity(),
      targetWheelSpeeds.rearRightMetersPerSecond);

  frontLeftMotor.set(MathUtil.clamp(flOutput, -1.0, 1.0));
  frontRightMotor.set(MathUtil.clamp(frOutput, -1.0, 1.0));
  backLeftMotor.set(MathUtil.clamp(blOutput, -1.0, 1.0));
  backRightMotor.set(MathUtil.clamp(brOutput, -1.0, 1.0));
}

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

  drive(fieldRelativeSpeeds);
}


  private Rotation2d getHeading() {
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
    odometry.update(
        getHeading(),
        getWheelPositions()
    );

    // Display pose if in sim; otherwise you'll want to do real odometry here.
    if (RobotBase.isSimulation() && sim != null) {
      field.setRobotPose(sim.getPose());
    }

  }

    @Override
  public void simulationPeriodic() {
    if (sim != null) {
      sim.update();
    }
  }

  public Pose2d getSimPose() {
    return (sim != null) ? sim.getPose() : new Pose2d();
  }

}
