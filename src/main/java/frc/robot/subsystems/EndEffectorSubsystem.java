// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;


public class EndEffectorSubsystem extends SubsystemBase {
  private final SparkMax m_intakeMotor;// the motor that feeds the coral inside the end effector
  private final SparkMax m_directionMotor; //the motor that makes it go up or down
  private final SparkMaxConfig config;

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem() {
    m_intakeMotor = new SparkMax(EndEffectorConstants.kEndEffectorIntakeCanId, MotorType.kBrushless);
    m_directionMotor = new SparkMax(EndEffectorConstants.kEndEffectorDirectionCanId, MotorType.kBrushless);
    
    config = new SparkMaxConfig();

    m_intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_directionMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Method to move the entire end effector structure up or down.
   * @param direction - The direction in which the end effector structure will move, with true moving the structure down, and false moving the structure up.
   */

  public void driveEndEffector(boolean direction){
    if(direction == true){
      m_directionMotor.set(EndEffectorConstants.kEndEffectorMaxSpeedDown); //moves the whole structure down
    }
    else if(direction == false){
      m_directionMotor.set(EndEffectorConstants.kEndEffectorMaxSpeedUp);
    }

  }

  /**
   * Method that drives the intake motors to go in or out.
   * @param direction - Describes the direction that the intake motor will go in. If direction is true, the intake motor will make the mechanism go inwards. False makes the mechanism go outwards.
   */

  public void driveIntake(boolean direction){
    if(direction == true){
      m_intakeMotor.set(EndEffectorConstants.kEndEffectorMaxSpeedIn);
    } else if(direction == false){
      m_intakeMotor.set(EndEffectorConstants.kEndEffectorMaxSpeedOut);
    }

  }

  /**
   * Method to stop a motor of the end effector system.
   * @param motor - The motor that will be stopped when the function is called. True is for the intake motor, False is for the structure motor.
   */
  public void stop(boolean motor){ // 1 is for intake, 2 is for direction motor
    if(motor == true){
      m_intakeMotor.set(0.0);
    } else {
      m_directionMotor.set(0.0);
    }
  }

}
