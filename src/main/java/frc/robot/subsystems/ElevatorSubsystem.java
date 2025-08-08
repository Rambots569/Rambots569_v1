// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.SubsystemConfigs;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;


public class ElevatorSubsystem extends SubsystemBase {
   public final SparkMax m_rightSparkMax;
   public final SparkMax m_leftSparkMax;

  private final SparkMaxConfig SparkMaxConfig;
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_rightSparkMax = new SparkMax(ElevatorConstants.kLeaderElevatorCanId, MotorType.kBrushless);
    m_leftSparkMax = new SparkMax(ElevatorConstants.kFollowerElevatorCanId, MotorType.kBrushless);

    SparkMaxConfig = SubsystemConfigs.defaultConfig;

   
    m_rightSparkMax.configure(SparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

/**
 * Method that controls the motors, and makes them drive the elevator system.
 * @param direction - Represents the direction in which the elevator will go; True is up, false is down.
 */

  public void driveElevator(double speed){
      m_rightSparkMax.set(speed);
      m_leftSparkMax.set(-(speed));
  }

  /** Stops the elevator motors. */
  public void stop(){
    m_rightSparkMax.set(0.0);
    m_leftSparkMax.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
