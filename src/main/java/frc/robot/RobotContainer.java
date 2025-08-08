// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.PS4Controller;
import java.io.File;
import java.util.function.DoubleSupplier;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import swervelib.SwerveInputStream;


public class RobotContainer {
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_robotDrive.getSwerveDrive(),
   () -> m_driverController.getLeftY(), 
   () -> m_driverController.getLeftX() * - 1)  //lambdas are being used to be able to modify the DoubleSupplier's value because it's stupid
    .withControllerRotationAxis(() -> m_driverController.getRightX() * -1) 
    .deadband(Constants.OIConstants.kDriveDeadband)
    .scaleTranslation(0.8)
    .allianceRelativeControl(false);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY).headingWhile(true);
  ChassisSpeeds velocity;

  Command driveCommandDirectAngle = m_robotDrive.driveCommandTwo(driveAngularVelocity);
  
  public RobotContainer() {
    configureBindings();
    //m_robotDrive.setDefaultCommand(
      //new RunCommand(
       // () ->
       // m_robotDrive.driveCommand(
        //  -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
         // -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
         // -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)),
        //m_robotDrive));
    m_robotDrive.setDefaultCommand(driveCommandDirectAngle);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
