// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.*;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.AgitatorConstants;

public class Agitators extends SubsystemBase {
  private SparkMax agitatorMotor;
    
  public Agitators() {
    agitatorMotor = new SparkMax(AgitatorConstants.kAgitatorPort, MotorType.kBrushless);
    SparkMaxConfig agitatorMotorConfig = new SparkMaxConfig();
    // configure agitator motor
    agitatorMotor.configure(agitatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setAgitators(double speed) {
    agitatorMotor.set(speed);
  }

  public void stopAgitators() {
    agitatorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    @Override
  public void simulationPeriodic() {
    
    }

}