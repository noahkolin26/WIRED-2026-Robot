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

import frc.robot.Constants.ThroatConstants;

public class Throat extends SubsystemBase {
  private SparkMax throatLeftMotor;
  private SparkMax throatRightMotor;
    
  public Throat() {
    throatLeftMotor = new SparkMax(ThroatConstants.kThroatLeftPort, MotorType.kBrushed);
    throatRightMotor = new SparkMax(ThroatConstants.kThroatRightPort, MotorType.kBrushed);

    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig
      .inverted(true);


    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
    rightMotorConfig
      .inverted(true)
      .follow(throatLeftMotor); //right is following left.  LEFT is LEADER

    // configure throat motors
    throatLeftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    throatRightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setThroatPower(double speed) {
    throatLeftMotor.set(speed);
  }

  public void stopThroat() {
    throatLeftMotor.set(0);
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