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

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private SparkMax intakeMotor;

  RelativeEncoder intakeMotorEnc;
    
  public Intake() {
    intakeMotor = new SparkMax(IntakeConstants.kIntakePort, MotorType.kBrushless);
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig
      .inverted(false);
    // configure intake motor
    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotorEnc = intakeMotor.getEncoder();
  }

  public void setIntake(double speed) {
    intakeMotor.set(speed);
    Telemetry.putDouble("Goal Intake Speed", speed);
  }

  public void stopIntake() {
    intakeMotor.set(0);
    Telemetry.putDouble("Goal Intake Speed", 0);
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
    Telemetry.putDouble("Actual Intake Speed", intakeMotorEnc.getVelocity());
  }

    @Override
  public void simulationPeriodic() {
    
    }

}