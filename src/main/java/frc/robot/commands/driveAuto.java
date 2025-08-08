// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class driveAuto extends Command {
  private final DriveSubsystem driveSubsytem;

  public driveAuto(DriveSubsystem driveSubsystem) {
    driveSubsytem = driveSubsystem;
    addRequirements(driveSubsytem);
  }

  @Override
  public void initialize() {
    // Any initialization logic if needed
  }

  @Override
  public void execute() {
    driveSubsytem.driveCommand(0.3, 0.0, 0.0, 0.0);
  }

  @Override
  public void end(boolean interrupted) {
    // If interrupted, stop the elevator
    driveSubsytem.driveCommand(0.0, 0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    // This command continues running until explicitly stopped by another command
    return false;
  }
}