// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.util.Telemetry;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.*;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ThroatConstants;

public class Throat extends SubsystemBase {
  private SparkMax throatLeftMotor;
  private SparkMax throatRightMotor;
  private RelativeEncoder throatLeftEnc;
  private RelativeEncoder throatRightEnc;
    
  public Throat() {
    throatLeftMotor = new SparkMax(ThroatConstants.kThroatLeftPort, MotorType.kBrushed);
    throatRightMotor = new SparkMax(ThroatConstants.kThroatRightPort, MotorType.kBrushed);

    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig
      .inverted(false);
    throatLeftEnc = throatLeftMotor.getEncoder();

    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
    rightMotorConfig
      .inverted(false)
      .follow(throatLeftMotor); //right is following left.  LEFT is LEADER
    throatRightEnc = throatRightMotor.getEncoder();

    // configure throat motors
    throatLeftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    throatRightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setThroatPower(double speed) {
    throatLeftMotor.set(speed);
    Telemetry.putDouble("Goal Throat Speed", speed);
  }

  public void stopThroat() {
    throatLeftMotor.set(0);
    Telemetry.putDouble("Goal Throat Speed", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Telemetry.putDouble("Throat Left Encoder", throatLeftEnc.getVelocity());
    Telemetry.putDouble("Throat Right Encoder", throatRightEnc.getVelocity());
  }

    @Override
  public void simulationPeriodic() {
    
    }

}