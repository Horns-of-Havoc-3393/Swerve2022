package frc.robot.commands;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveBaseSubsystem;

public class SimpleAutoCmd extends CommandBase{

    SwerveBaseSubsystem swerveBaseSubsystem;
    long startingTime;

    BooleanSubscriber redSub;

    public SimpleAutoCmd(SwerveBaseSubsystem swerveBaseSubsystem) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable FMSInfo = inst.getTable("/FMSInfo");
        redSub = FMSInfo.getBooleanTopic("IsRedAlliance").subscribe(false);

        this.swerveBaseSubsystem = swerveBaseSubsystem;
        addRequirements(swerveBaseSubsystem);
        
        startingTime = System.currentTimeMillis();
    }

    @Override
    public void execute(){
        if(System.currentTimeMillis()<startingTime+1000){
            if(redSub.get()){
                swerveBaseSubsystem.setFieldOriented(-1, 0.0, 0.0, true);
            }else{
                swerveBaseSubsystem.setFieldOriented(-1, 0.0, 0.0, true);
            }
        }else if(System.currentTimeMillis()<startingTime+2500){
            if(redSub.get()){
                swerveBaseSubsystem.setFieldOriented(1, 0.0, 0.0, true);
            }else{
                swerveBaseSubsystem.setFieldOriented(1, 0.0, 0.0, true);
            }
        }else if(System.currentTimeMillis()<startingTime+6500){
            if(redSub.get()){
                swerveBaseSubsystem.setFieldOriented(-1.5, 0.0, 0.0, true);
            }else{
                swerveBaseSubsystem.setFieldOriented(-1.5, 0.0, 0.0, true);
            }

        }else{
            swerveBaseSubsystem.setRelative(0.0,0.0,0.0);
        }
    }
}
