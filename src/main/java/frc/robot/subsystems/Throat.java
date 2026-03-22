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

/*
 * commit message "early" 3/22
 * > Problem: PathPlanner keeps getting "hung up" on certain parts of the autonomous
 * > Suspicion: The first command doesn't finish running so it can't move onto the next one, hence why Parallel works
 * > Testing:
 *      *Assuming* that the Simulator "accurately" depicts robot movements and that the motors will move as Elastic pretends...
 *      - Changing all the commands to InstantCommands (runs within one scheduler loop and finishes) made the autonomous finish (don't know what it did before though in the sim)
 *      - But the value of the "throat" motor isn't changing
 * > GPT:
 *      - now, instead of setting the motor via a method, FOR THIS SUBSYSTEM ONLY SO FAR we store the GoalSpeed as a static variable.
 *      - every run of the scheduler, it continuously sets the motor to whatever GoalSpeed is
 *      - this supposedly makes it easy to set the goalSpeed separately via different commands?
 * > Results:
 *      - so apparently the SetThroatInstant command was automatically setting it back to 0 when it ended
 *      - I REMOVED THAT FROM BOTH THROAT COMMANDS, THE THROAT WON'T STOP UNTIL YOU SPECIFICALLY TELL IT TO GO TO ZERO
 *      - according to the telemetry in Elastic, this correctly works for that one autonomous at least
 *      - so if we don't crash into anything the HD-L+MLD-L auto (Soon to be renamed) will shoot medium, go NZ to get some, shoot, then sit by NZ?? (cool if true)
 */
public class Throat extends SubsystemBase {
  private SparkMax throatLeftMotor;
  private SparkMax throatRightMotor;
  private RelativeEncoder throatLeftEnc;
  private RelativeEncoder throatRightEnc;

  public double goalSpeed = 0.0;
    
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
    goalSpeed = speed;
  }

  public void stopThroat() {
    goalSpeed = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    throatLeftMotor.set(goalSpeed);
    Telemetry.putDouble("Goal Throat Speed", goalSpeed);
    Telemetry.putDouble("Throat Left Encoder", throatLeftEnc.getVelocity());
    Telemetry.putDouble("Throat Right Encoder", throatRightEnc.getVelocity());
  }

    @Override
  public void simulationPeriodic() {

  }
}