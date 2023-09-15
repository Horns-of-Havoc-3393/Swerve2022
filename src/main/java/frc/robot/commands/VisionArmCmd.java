package frc.robot.commands;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveBaseSubsystem;

public class VisionArmCmd extends CommandBase{
    

    SwerveBaseSubsystem swerveBaseSubsystem;

    BooleanSubscriber redSub;
    
    DoubleSubscriber visX;
    DoubleSubscriber visY;
    DoublePublisher pitchOutPub;
    DoublePublisher thetaOutPub;

    ArmSubsystem armSubsystem;


    public VisionArmCmd(ArmSubsystem armSubsystem, SwerveBaseSubsystem swerveBaseSubsystem){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable FMSInfo = inst.getTable("/FMSInfo");
        redSub = FMSInfo.getBooleanTopic("IsRedAlliance").subscribe(false);
        NetworkTable vision = inst.getTable("/vision");
        visX = vision.getDoubleTopic("objectX").subscribe(0.0);
        visY = vision.getDoubleTopic("objectY").subscribe(0.0);
        pitchOutPub = vision.getDoubleTopic("pitchOut").publish();
        thetaOutPub = vision.getDoubleTopic("thetaOut").publish();

        

        this.swerveBaseSubsystem = swerveBaseSubsystem;

  

        addRequirements(swerveBaseSubsystem);

        this.armSubsystem = armSubsystem;

        addRequirements(this.armSubsystem);
        return;
    }

    
    @Override
    public void execute() {
        double xAxis = RobotContainer.driveController.getRawAxis(0);
        double yAxis = -RobotContainer.driveController.getRawAxis(1);
        double thetaAxis = (((visX.get()/320*2)-1)*0.5);
        thetaOutPub.set(thetaAxis);



        // if(redSub.get()){
        //     swerveBaseSubsystem.setFieldOriented(-yAxis, -xAxis, thetaAxis, false);
        // }else{
        //     swerveBaseSubsystem.setFieldOriented(yAxis, xAxis, thetaAxis, false);
        // }
        // int povAngle = RobotContainer.armController.getPOV();

        // if(RobotContainer.armController.getRawButton(5)){
        //     armSubsystem.sol1(true);
        // }else{
        //     armSubsystem.sol1(false);
        // }
        // if(RobotContainer.armController.getRawButton(6)){
        //     armSubsystem.sol2(true);
        // }else{
        //     armSubsystem.sol2(false);
        // }


        // armSubsystem.extendArm(RobotContainer.armController.getRawAxis(5));
        // armSubsystem.turnArm(((visY.get()+0.1)/384*2-1)/3);
        // pitchOutPub.set((((visY.get()+0.1)/384*2-1))/3);

        // armSubsystem.setIntakeSpeed((RobotContainer.armController.getRawAxis(2))-(RobotContainer.armController.getRawAxis(3)));

        return;
    }
}
