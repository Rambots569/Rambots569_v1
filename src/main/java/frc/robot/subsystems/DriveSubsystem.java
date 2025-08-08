// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.imu.NavXSwerve;
import swervelib.imu.*;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.SwerveDrive.*;
import swervelib.math.SwerveMath;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {
  SwerveDrive swerveDrive;
  File swerveDir = new File(Filesystem.getDeployDirectory(), "swerve");
  /** Creates a new driveSubsystem. */
  public DriveSubsystem() {
      try
      {
        swerveDrive = new SwerveParser(swerveDir).createSwerveDrive(DriveConstants.kMaxSpeedMetersPerSecond);
        // Alternative method if you don't want to supply the conversion factor via JSON files.
        // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
      } catch (IOException e)
      {
        System.err.println(e);
      }
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
    /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(double translationX, double translationY, double headingX,
                              double headingY)
  {
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX,
                                                                                 translationY), 0.8);

      // Make the robot move
      swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX,
                                                                      headingY,
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(double translationX, double translationY, double angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX * swerveDrive.getMaximumChassisVelocity(),
                                          translationY * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX * swerveDrive.getMaximumChassisAngularVelocity(),
                        false,
                        false);
    });
  }

  public SwerveDrive getSwerveDrive(){
    return swerveDrive;
  }
  public Command driveCommandTwo(SwerveInputStream stream)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(stream.get());
    });
  }
}
