// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.swerveModConstants.driveConstants;
import frc.robot.subsystems.SwerveBaseSubsystem;
import frc.robot.subsystems.SwerveModSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SwerveAbsCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final SwerveBaseSubsystem m_swerveBaseSubsystem;


  

  public SwerveAbsCmd(SwerveBaseSubsystem swerveBaseSubsystem) {

    this.m_swerveBaseSubsystem = swerveBaseSubsystem;

  

    addRequirements(m_swerveBaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xAxis = RobotContainer.driveController.getRawAxis(0);
    double yAxis = RobotContainer.driveController.getRawAxis(1);
    double thetaAxis = RobotContainer.driveController.getRawAxis(4);

    System.out.print(m_swerveBaseSubsystem.getBaseAngle());


    m_swerveBaseSubsystem.setFieldOrented(xAxis, yAxis, thetaAxis);


  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
