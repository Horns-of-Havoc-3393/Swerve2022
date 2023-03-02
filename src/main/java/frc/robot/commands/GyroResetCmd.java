package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PositioningSubsystem;
import frc.robot.subsystems.SwerveBaseSubsystem;

public class GyroResetCmd extends CommandBase {

    private PositioningSubsystem positioningSubsystem;

    private Boolean end = false;

    public GyroResetCmd(){

        addRequirements(RobotContainer.positionSubsystem);

        this.positioningSubsystem = RobotContainer.positionSubsystem;

        end = true;

    }

    @Override
    public boolean isFinished(){
        return end;
    }

    @Override
    public void initialize(){
        positioningSubsystem.zeroHeading();
    }

}
