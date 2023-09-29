// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveBaseSubsystem;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SwerveAbsCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final SwerveBaseSubsystem m_swerveBaseSubsystem;

  private BooleanSubscriber redSub;

 
  

  public SwerveAbsCmd(SwerveBaseSubsystem swerveBaseSubsystem) {

    // NetworkTable init for telemetry
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable FMSInfo = inst.getTable("/FMSInfo");
    redSub = FMSInfo.getBooleanTopic("IsRedAlliance").subscribe(false);

    this.m_swerveBaseSubsystem = swerveBaseSubsystem;

  

    addRequirements(m_swerveBaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Sends the axes to the swerve subsystem
    double xAxis = RobotContainer.driveController.getRawAxis(0);
    double yAxis = -RobotContainer.driveController.getRawAxis(1);
    double thetaAxis = RobotContainer.driveController.getRawAxis(4);



    if(redSub.get()){
      m_swerveBaseSubsystem.setFieldOriented(-yAxis, -xAxis, thetaAxis, false);
    }else{
      m_swerveBaseSubsystem.setFieldOriented(yAxis, xAxis, thetaAxis, false);
    }


  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
