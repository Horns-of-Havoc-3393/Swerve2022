// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.GyroResetCmd;
import frc.robot.commands.SimpleAutoCmd;
import frc.robot.commands.SwerveAbsCmd;
import frc.robot.commands.TurnPIDUpdateCmd;
import frc.robot.subsystems.PositioningSubsystem;
import frc.robot.subsystems.SwerveBaseSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  
  public static final PositioningSubsystem positionSubsystem = new PositioningSubsystem();

  public static final SwerveBaseSubsystem swerveBaseSubsystem = new SwerveBaseSubsystem(positionSubsystem);

  public static final SwerveAbsCmd swerveAbsCommand = new SwerveAbsCmd(swerveBaseSubsystem);

  public static final XboxController driveController = new XboxController(0);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton button_a = new JoystickButton(driveController, 1);
    JoystickButton button_b = new JoystickButton(driveController, 2);

    button_a.onTrue(new GyroResetCmd());
    button_b.onTrue(new TurnPIDUpdateCmd(swerveBaseSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new FollowPathCmd(swerveBaseSubsystem);
    return new SimpleAutoCmd(swerveBaseSubsystem);
  }
}
