// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberCommand extends Command {
  /** Creates a new ClimberCommand. */
  private int direction;
  private ClimberSubsystem m_ClimberSubsystem;

  public ClimberCommand(ClimberSubsystem sub, int dir) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberSubsystem = sub;
    direction = dir;
    addRequirements(m_ClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(direction == 1){
      m_ClimberSubsystem.driveClimber(true);
    } else if(direction == 2) {
      m_ClimberSubsystem.driveClimber(false);
    } else {
      m_ClimberSubsystem.stop();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
