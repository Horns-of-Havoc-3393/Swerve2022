package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveBaseSubsystem;

public class TurnPIDUpdateCmd extends CommandBase {

    private SwerveBaseSubsystem swerveBaseSubsystem;

    private Boolean end = false;

    public TurnPIDUpdateCmd(SwerveBaseSubsystem m_swerveBaseSubsystem){

        addRequirements(m_swerveBaseSubsystem);

        this.swerveBaseSubsystem = m_swerveBaseSubsystem;

        end = true;

    }

    @Override
    public boolean isFinished(){
        return end;
    }

    @Override
    public void initialize(){
        swerveBaseSubsystem.updateTurnPIDs();
    }

}
