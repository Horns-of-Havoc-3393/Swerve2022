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

        
    //     if(RobotContainer.armController.getRawButton(6)){
    //         armSubsystem.sol2(true);
    //     }else{
    //         armSubsystem.sol2(false);
    //     }
    //     if(RobotContainer.armController.getRawButton(5)){
    //         armSubsystem.sol1(true);
    //         armSubsystem.sol2(true);
    //     }else{
    //         armSubsystem.sol1(false);
    //     }

            

    //     armSubsystem.extendArm(RobotContainer.armController.getRawAxis(5));
    //     armSubsystem.turnArm(RobotContainer.armController.getRawAxis(1));

    //     armSubsystem.setIntakeSpeed(((RobotContainer.armController.getRawAxis(2))-(RobotContainer.armController.getRawAxis(3))));

    //     return;
    }
}
