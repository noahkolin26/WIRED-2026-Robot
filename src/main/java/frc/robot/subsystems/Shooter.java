// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.util.Telemetry;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

// https://www.chiefdelphi.com/t/open-source-shoot-on-the-move-sotm-solver-ball-physics-sim-3-java-files-drop-in/516109

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private TalonFX shootMotor;

  private RelativeEncoder shootMotorEnc;

  private double currentSpeed = 0.0;
    
  public Shooter() {
    shootMotor = new TalonFX(ShooterConstants.kShooterPort, CANBus.roboRIO());
    MotorOutputConfigs config = new MotorOutputConfigs();
    config
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake);
  }

  public void setShooter(double speed) {
    shootMotor.set(speed);
    currentSpeed = speed;
    Telemetry.putDouble("Goal Shooter Speed", speed);
  }

  public void stopShooter() {
    shootMotor.set(0);
    currentSpeed = 0.0;
    Telemetry.putDouble("Goal Shooter Speed", 0);
  }

  public double getPower() {
    return currentSpeed;
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
    Telemetry.putDouble("Actual Shooter Speed", shootMotorEnc.getVelocity());
  }

    @Override
  public void simulationPeriodic() {
    
    }

}