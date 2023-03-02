package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PositioningSubsystem;
import frc.robot.subsystems.SwerveBaseSubsystem;

public class AutoBalance extends CommandBase {

    private PositioningSubsystem positioningSubsystem;
    private SwerveBaseSubsystem swerveBaseSubsystem;

    private PIDController tiltPID;

    public AutoBalance(PositioningSubsystem positioningSubsystem, SwerveBaseSubsystem swerveBaseSubsystem) {

        addRequirements(swerveBaseSubsystem);

        this.positioningSubsystem = positioningSubsystem;
        this.swerveBaseSubsystem = swerveBaseSubsystem;
        
    }

    @Override
    public void execute() {
        swerveBaseSubsystem.setRelative(0, Math.min(positioningSubsystem.getQuaternion().getX()*-15.5,2), 0);
    }
}
