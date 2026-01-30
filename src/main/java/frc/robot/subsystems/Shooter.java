// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.*;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;;

public class Shooter extends SubsystemBase {
  private SparkMax shootMotor;
    
  public Shooter() {
    shootMotor = new SparkMax(ShooterConstants.kShooterPort, MotorType.kBrushless);
    SparkMaxConfig shootMotorConfig = new SparkMaxConfig();
    // configure agitator motor
    shootMotor.configure(shootMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setShooter(double speed) {
    shootMotor.set(speed);
  }

  public void stopShooter() {
    shootMotor.set(0);
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