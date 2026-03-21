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

  private double currentSpeed = 0.0;
  private double actualSpeed = 0.0;
  private double currentRPS = 0.0;

  private int shooterPowerIndex = 0;
    
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
    if(speed == 0) {
      shooterPowerIndex = 0;
    }
    Telemetry.putDouble("Goal Shooter Speed", speed);
  }

  public void stopShooter() {
    shootMotor.set(0);
    currentSpeed = 0.0;
    Telemetry.putDouble("Goal Shooter Speed", 0);
  }

  public void updatePowerFromIndex() {
    if(shooterPowerIndex == 0) {
      setShooter(ShooterConstants.shootPowerSHORT);
    } else if (shooterPowerIndex == 1) {
      setShooter(ShooterConstants.shootPowerMEDIUM);
    } else if (shooterPowerIndex == 2) {
      setShooter(ShooterConstants.shootPowerLONG);
    } else if (shooterPowerIndex == 3) {
      setShooter(1.0);
    }
  }

  public void advanceIndex() {
    shooterPowerIndex = (shooterPowerIndex + 1) % 4;
  }

  public void reverseIndex() {
    shooterPowerIndex = (shooterPowerIndex - 1 + 4) % 4;
  }

  public double getPower() {
    return currentSpeed;
  }

  public double getActualSpeed() {
    return actualSpeed;
  }

  public double getCurrentRPS() {
    return currentRPS;
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
    currentRPS = shootMotor.getVelocity().getValueAsDouble();
    actualSpeed = currentRPS / 106;
    Telemetry.putBoolean("Shooter Near Power?", Math.abs(currentSpeed - actualSpeed) < 0.1);
    Telemetry.putDouble("Actual Shooter Speed", actualSpeed);
    Telemetry.putDouble("Current Shooter RPS", currentRPS);
    
    Telemetry.putBoolean("Shooter Off", currentSpeed == 0);
    Telemetry.putBoolean("Shooter Short", currentSpeed == ShooterConstants.shootPowerSHORT);
    Telemetry.putBoolean("Shooter Medium", currentSpeed == ShooterConstants.shootPowerMEDIUM);
    Telemetry.putBoolean("Shooter Long", currentSpeed == ShooterConstants.shootPowerLONG);
    Telemetry.putBoolean("Shooter Full", currentSpeed == 1);
  }

    @Override
  public void simulationPeriodic() {
    
    }

}