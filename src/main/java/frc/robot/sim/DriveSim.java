package frc.robot.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSim {
  private final SparkMaxSim flSparkSim, frSparkSim, rlSparkSim, rrSparkSim;
  private final SparkRelativeEncoderSim flEncSim, frEncSim, rlEncSim, rrEncSim;

  // Per-wheel motor physics
  private final DCMotorSim flMotorSim, frMotorSim, rlMotorSim, rrMotorSim;

  private final MecanumDriveKinematics kinematics;

  private Pose2d pose = new Pose2d();
  private final SimGyro gyro;

  public DriveSim(
      SparkMax fl, SparkMax fr, SparkMax rl, SparkMax rr,
      SimGyro gyro
  ) {
    this.gyro = gyro;

    var neo = DCMotor.getNEO(1);

    flSparkSim = new SparkMaxSim(fl, neo);
    frSparkSim = new SparkMaxSim(fr, neo);
    rlSparkSim = new SparkMaxSim(rl, neo);
    rrSparkSim = new SparkMaxSim(rr, neo);

    flEncSim = new SparkRelativeEncoderSim(fl);
    frEncSim = new SparkRelativeEncoderSim(fr);
    rlEncSim = new SparkRelativeEncoderSim(rl);
    rrEncSim = new SparkRelativeEncoderSim(rr);

    // MotorSim plant: includes gearbox + inertia. Gearing is motor->wheel.
    flMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(neo, Constants.DriveConstants.kWheelMoi, Constants.DriveConstants.kDriveGearing),
        neo
    );
    frMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(neo, Constants.DriveConstants.kWheelMoi, Constants.DriveConstants.kDriveGearing),
        neo
    );
    rlMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(neo, Constants.DriveConstants.kWheelMoi, Constants.DriveConstants.kDriveGearing),
        neo
    );
    rrMotorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(neo, Constants.DriveConstants.kWheelMoi, Constants.DriveConstants.kDriveGearing),
        neo
    );

    kinematics = new MecanumDriveKinematics(
        new Translation2d(+Constants.DriveConstants.kHalfWheelbaseMeters, +Constants.DriveConstants.kHalfTrackwidthMeters), // FL
        new Translation2d(+Constants.DriveConstants.kHalfWheelbaseMeters, -Constants.DriveConstants.kHalfTrackwidthMeters), // FR
        new Translation2d(-Constants.DriveConstants.kHalfWheelbaseMeters, +Constants.DriveConstants.kHalfTrackwidthMeters), // RL
        new Translation2d(-Constants.DriveConstants.kHalfWheelbaseMeters, -Constants.DriveConstants.kHalfTrackwidthMeters)  // RR
    );
  }

   public Pose2d getPose() {
    return pose;
  }

  /** Call from DriveSubsystem.simulationPeriodic() */
  public void update() {
    final double dt = Constants.DriveConstants.kDtSeconds;
    final double vbus = RobotController.getBatteryVoltage();

    // 1) Read commanded output from the SPARK sims, convert to volts
    double flVolts = flSparkSim.getAppliedOutput() * vbus;
    double frVolts = frSparkSim.getAppliedOutput() * vbus;
    double rlVolts = rlSparkSim.getAppliedOutput() * vbus;
    double rrVolts = rrSparkSim.getAppliedOutput() * vbus;

    // 2) Step motor sims
    flMotorSim.setInputVoltage(flVolts);
    frMotorSim.setInputVoltage(frVolts);
    rlMotorSim.setInputVoltage(rlVolts);
    rrMotorSim.setInputVoltage(rrVolts);

    flMotorSim.update(dt);
    frMotorSim.update(dt);
    rlMotorSim.update(dt);
    rrMotorSim.update(dt);

    // 3) Motor angular velocity -> wheel linear speed
    double vFL = flMotorSim.getAngularVelocityRadPerSec() * Constants.DriveConstants.kWheelRadiusMeters / DriveConstants.kDriveGearing;
    double vFR = frMotorSim.getAngularVelocityRadPerSec() * Constants.DriveConstants.kWheelRadiusMeters / DriveConstants.kDriveGearing;
    double vRL = rlMotorSim.getAngularVelocityRadPerSec() * Constants.DriveConstants.kWheelRadiusMeters / DriveConstants.kDriveGearing;
    double vRR = rrMotorSim.getAngularVelocityRadPerSec() * Constants.DriveConstants.kWheelRadiusMeters / DriveConstants.kDriveGearing;

    // 4) Wheel speeds -> chassis speeds (robot frame)
    ChassisSpeeds chassis =
        kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(vFL, vFR, vRL, vRR));

    // 5) Integrate pose (robot-relative twist)
    pose = pose.exp(new Twist2d(
        chassis.vxMetersPerSecond * dt,
        chassis.vyMetersPerSecond * dt,
        chassis.omegaRadiansPerSecond * dt
    ));

    // 6) Update simulated gyro to match pose heading
    gyro.setAngleDegrees(pose.getRotation().getDegrees());

    // 7) Update encoder sim (NEO integrated encoder is motor-side)
    // Motor position/velocity from motor sims are motor-shaft values.
    double r = DriveConstants.kWheelRadiusMeters;

    double flPosMeters = flMotorSim.getAngularPositionRad() * r;
    double frPosMeters = frMotorSim.getAngularPositionRad() * r;
    double rlPosMeters = rlMotorSim.getAngularPositionRad() * r;
    double rrPosMeters = rrMotorSim.getAngularPositionRad() * r;

    double flVelMps = flMotorSim.getAngularVelocityRadPerSec() * r;
    double frVelMps = frMotorSim.getAngularVelocityRadPerSec() * r;
    double rlVelMps = rlMotorSim.getAngularVelocityRadPerSec() * r;
    double rrVelMps = rrMotorSim.getAngularVelocityRadPerSec() * r;

    flEncSim.setPosition(flPosMeters);
    frEncSim.setPosition(frPosMeters);
    rlEncSim.setPosition(rlPosMeters);
    rrEncSim.setPosition(rrPosMeters);

    flEncSim.setVelocity(flVelMps);
    frEncSim.setVelocity(frVelMps);
    rlEncSim.setVelocity(rlVelMps);
    rrEncSim.setVelocity(rrVelMps);

    // 8) Let the SPARK sim advance too (keeps its internal sim state consistent)
    double flWheelRPM = flMotorSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
    double frWheelRPM = frMotorSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
    double rlWheelRPM = rlMotorSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
    double rrWheelRPM = rrMotorSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);

    double flMotorRPM = flWheelRPM * DriveConstants.kDriveGearing;
    double frMotorRPM = frWheelRPM * DriveConstants.kDriveGearing;
    double rlMotorRPM = rlWheelRPM * DriveConstants.kDriveGearing;
    double rrMotorRPM = rrWheelRPM * DriveConstants.kDriveGearing;

    flSparkSim.iterate(flMotorRPM, vbus, dt);
    frSparkSim.iterate(frMotorRPM, vbus, dt);
    rlSparkSim.iterate(rlMotorRPM, vbus, dt);
    rrSparkSim.iterate(rrMotorRPM, vbus, dt);
  }
}
