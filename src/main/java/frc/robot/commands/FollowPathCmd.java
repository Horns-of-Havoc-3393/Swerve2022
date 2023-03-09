package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.swerveModConstants.autoConstants;
import frc.robot.subsystems.SwerveBaseSubsystem;

public class FollowPathCmd extends CommandBase{

    
    Trajectory trajectory = new Trajectory();

    long startTime = System.currentTimeMillis();

    DoublePublisher elapsedPub;


    SwerveBaseSubsystem swerveBaseSubsystem;

    Field2d field;


    public FollowPathCmd(SwerveBaseSubsystem swerveBaseSubsystem, String pathName){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("/debug/auto");
        elapsedPub = table.getDoubleTopic("elapsed").publish();

        field = new Field2d();


        this.swerveBaseSubsystem = swerveBaseSubsystem;
        addRequirements(swerveBaseSubsystem);

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/output/" + pathName);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathName, ex.getStackTrace());
        }

    }

    @Override
    public void execute() {

        double elapsed = (System.currentTimeMillis()-startTime);
        elapsed = elapsed/1000.0;
        elapsedPub.set(elapsed);
        Trajectory.State point = trajectory.sample((elapsed)+autoConstants.lookAhaid);
        swerveBaseSubsystem.toPoint(point.poseMeters);
        field.setRobotPose(point.poseMeters);
        SmartDashboard.putData("PathTarget", field);
    }
}
