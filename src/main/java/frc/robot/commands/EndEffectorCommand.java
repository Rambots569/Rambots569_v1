// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorCommand extends Command {
  /** Creates a new EndEffectorCommand. */
  private EndEffectorSubsystem m_EndEffectorSubsystem;
  private int intakeDirection; //same as elevator, 1 for in , 2 for out, 3 for stop (for intake)
  private int direction; // direction for whole structure, 1 for down, 2 for up, 3 for stop
  private boolean intakeOrStructure;


  public EndEffectorCommand(EndEffectorSubsystem sub, int dir, boolean whichMotor) {
    // Use addRequirements() here to declare subsystem dependencies.
    //whichMotor - true is for intake, false is for whole structure
    m_EndEffectorSubsystem = sub;
    intakeDirection = dir;
    direction = dir;
    intakeOrStructure = whichMotor;
    addRequirements(m_EndEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean directionAsBoolean = false;
    if(intakeOrStructure == true){ //for intake
      if(intakeDirection == 3){
        m_EndEffectorSubsystem.stop(true);
      } else if(intakeDirection == 1){
          directionAsBoolean = true;
          m_EndEffectorSubsystem.driveIntake(directionAsBoolean);
      } else {
        m_EndEffectorSubsystem.driveIntake(directionAsBoolean);
      }
    } else { // for moving whole structure
      if(direction == 3){
        m_EndEffectorSubsystem.stop(false);
      } else if(direction == 1){
        directionAsBoolean = true;
        m_EndEffectorSubsystem.driveEndEffector(directionAsBoolean);
      } else {
        m_EndEffectorSubsystem.driveEndEffector(directionAsBoolean);
      }
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
