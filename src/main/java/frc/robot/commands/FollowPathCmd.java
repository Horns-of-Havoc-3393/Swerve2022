package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.swerveModConstants.autoConstants;
import frc.robot.subsystems.SwerveBaseSubsystem;

public class FollowPathCmd extends CommandBase{

    
    Trajectory trajectory = new Trajectory();

    long startTime = System.currentTimeMillis();


    SwerveBaseSubsystem swerveBaseSubsystem;


    public FollowPathCmd(SwerveBaseSubsystem swerveBaseSubsystem, String pathName){

        this.swerveBaseSubsystem = swerveBaseSubsystem;

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/output/" + pathName);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathName, ex.getStackTrace());
        }

    }

    @Override
    public void execute() {

        long elapsed = System.currentTimeMillis()-startTime;
        Trajectory.State point = trajectory.sample((elapsed/1000)+autoConstants.lookAhaid);
        swerveBaseSubsystem.toPoint(point.poseMeters);
    }
}
