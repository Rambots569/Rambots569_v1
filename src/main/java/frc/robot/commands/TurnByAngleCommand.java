// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnByAngleCommand extends Command {
  private DriveSubsystem m_driveSubsystem;
  private boolean kDriveDirection;
  
  /** Creates a new TurnByAngleCommand. */
  public TurnByAngleCommand(DriveSubsystem drive, boolean direction) {
    m_driveSubsystem = drive;
    kDriveDirection = direction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * true - turn 90 degrees to the left
     * false - turn 90 degrees to the right
     */
    if(kDriveDirection == true){
      m_driveSubsystem.driveCommand(0.0, 0.0, 90.0, 0.0);
    } else if(kDriveDirection == false){
      m_driveSubsystem.driveCommand(0.0, 0.0, -90.0, -90.0);
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
