package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class DriverArmCmd extends CommandBase{


    ArmSubsystem armSubsystem;


    public DriverArmCmd(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;

        addRequirements(this.armSubsystem);
        return;
    }

    
    @Override
    public void execute() {
        int povAngle = RobotContainer.driveController.getPOV();

        if(RobotContainer.driveController.getRawButton(5)){
            armSubsystem.sol1(true);
        }else{
            armSubsystem.sol1(false);
        }
        if(RobotContainer.driveController.getRawButton(6)){
            armSubsystem.sol2(true);
        }else{
            armSubsystem.sol2(false);
        }

        if(povAngle == -1){
            armSubsystem.stop();
            return;
        }

        // if(povAngle<120 && povAngle>40){
        //     armSubsystem.extendArm(0.4);
        // }else if(povAngle>220 && povAngle<320){
        //     armSubsystem.extendArm(-0.4);
        // }

        if(povAngle<50 || povAngle>310){
            armSubsystem.turnArm(-0.6);
        }else if(povAngle>110 && povAngle<230){
            armSubsystem.turnArm(0.4);
        }
        return;
    }
}
