// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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
    public static final double wheelDiameter = 0.1524; // 6 inches
    public static final double gearRatio = 10.71;
    public static final double kDriveGearing = 10.71; // motor rotations per 1 wheel rotation (example)

    // Encoder conversion factors (NEO integrated encoder reports motor rotations/RPM)
    public static final double kMotorRotationsToWheelMeters =
        (2.0 * Math.PI * kWheelRadiusMeters) / kDriveGearing;
    public static final double kMotorRPMToWheelMps =
        (kMotorRotationsToWheelMeters / 60.0);

    // Sim tuning (rough but works)
    public static final double kWheelMoi = 0.001; // kg*m^2 (rough guess)
    public static final double kDtSeconds = 0.02;

    public static final double metersPerRotation = Math.PI * wheelDiameter / gearRatio;

    public static final double MAX_WHEEL_SPEED = 4.5; // meters/sec
  }
  
  public static class IntakeConstants {
    public static final int kIntakePort = 5;

    public static final double defaultIntakePower = 0.4;
  }

  public static class AgitatorConstants {
    public static final int kAgitatorPort = 6;

    public static final double defaultAgitatorPower = 0.4;
  }

  public static class ShooterConstants {
    public static final int kShooterPort = 7;

    public static final double defaultShooterPower = 0.4;
  }

  public static final double kMecanumSlewRate = 1000;
  public static final double kMecanumRotateSlewRate = 1000;
}
