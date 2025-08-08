// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final double m_ElevatorSpeed; // The direction that will be used for driving the elevator up or down (1 is up, 2 is down, 3 is stop)

  public ElevatorCommand(ElevatorSubsystem elevator, double speed) {
    m_ElevatorSubsystem = elevator;
    m_ElevatorSpeed = speed;
    addRequirements(m_ElevatorSubsystem);
  }

  @Override
  public void initialize() {
    // Any initialization logic if needed
  }

  @Override
  public void execute() {
    System.out.println("Elevator Command Executing: " + m_ElevatorSpeed);
      m_ElevatorSubsystem.driveElevator(m_ElevatorSpeed);

  }

  @Override
  public void end(boolean interrupted) {
    // If interrupted, stop the elevator
    m_ElevatorSubsystem.m_rightSparkMax.set(0);
    m_ElevatorSubsystem.m_leftSparkMax.set(0);
  }

  @Override
  public boolean isFinished() {
    // This command continues running until explicitly stopped by another command
    return false;
  }
}