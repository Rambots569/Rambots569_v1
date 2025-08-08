// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class ClimberSubsystem extends SubsystemBase {
  private SparkMax m_climberMotor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_climberMotor = new SparkMax(ClimberConstants.kClimberMotorCanId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

/**
 * Method to move the climber mechanism up or down, so that the robot can hang on the cage.
 * @param direction - Represents the direction in which the mechanism will move, with true being up, and false being down.
*/

  public void driveClimber(boolean direction){
    
    if(direction == true){ 
      m_climberMotor.set(ClimberConstants.kClimberMotorSpeedUp);
    } else if(direction == false){
      m_climberMotor.set(ClimberConstants.kClimberMotorSpeedDown);
    }
  }

  /** Stops the climbing mechanism motor. */
  public void stop(){
    m_climberMotor.set(0);
  }
}
