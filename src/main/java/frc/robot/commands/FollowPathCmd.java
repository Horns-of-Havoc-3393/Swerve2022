package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
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

    // initialized a bunch of variables
    Trajectory[] trajectories = new Trajectory[autoConstants.selectedPath.length]; // list of individual trajectories

    long startTime = System.currentTimeMillis();

    DoublePublisher elapsedPub;
    IntegerPublisher selectedPub;

    BooleanSubscriber redSub;


    SwerveBaseSubsystem swerveBaseSubsystem;

    Field2d field;

    int selectedPath;


    public FollowPathCmd(SwerveBaseSubsystem swerveBaseSubsystem){

        // NetworkTable initialization for telemetry
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("/debug/auto");
        elapsedPub = table.getDoubleTopic("elapsed").publish();
        selectedPub = table.getIntegerTopic("SelectedPath").publish();

        NetworkTable FMSInfo = inst.getTable("/FMSInfo");
        redSub = FMSInfo.getBooleanTopic("IsRedAlliance").subscribe(false);

        field = new Field2d();


        this.swerveBaseSubsystem = swerveBaseSubsystem; // set a local instance of the swerve subsystem
        addRequirements(swerveBaseSubsystem);

        // Adds all of the paths specified in constants to the list of paths
        for(int i=0; i<autoConstants.selectedPath.length; i++){
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/output/" + autoConstants.selectedPath[i] + ".wpilib.json");
                trajectories[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + autoConstants.selectedPath[i] + ".wpilib.json", ex.getStackTrace());
            }
        }

        initPath(0);

    }

    // Sets the current path and sets the position of the robot to the first point of the trajectory
    private void initPath(int path){
        selectedPub.set(path);
        swerveBaseSubsystem.setPosition(trajectories[path].sample(0).poseMeters.getX(), trajectories[path].sample(0).poseMeters.getY());
        selectedPath = path;
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {

        // Finds elapsed time in seconds
        double elapsed = (System.currentTimeMillis()-startTime);
        elapsed = elapsed/1000.0;
        elapsedPub.set(elapsed);
        
        // Gets point on trajectory x seconds ahaid
        Trajectory.State point = trajectories[selectedPath].sample((elapsed)+autoConstants.lookAhaid);
        Rotation2d endRotation = trajectories[selectedPath].sample(trajectories[selectedPath].getTotalTimeSeconds()).poseMeters.getRotation();

        Pose2d newPose = new Pose2d(point.poseMeters.getTranslation(), endRotation);

        // inverts everything for when you are on the opposite team
        if(redSub.get()){
            newPose = new Pose2d(16.54-newPose.getX(), newPose.getY(), newPose.getRotation());
        }


        swerveBaseSubsystem.toPoint(newPose); // tells the robot to go to the point
        field.setRobotPose(newPose);
        SmartDashboard.putData("PathTarget", field);

        if(elapsed>(trajectories[selectedPath].getTotalTimeSeconds())){
            initPath(selectedPath+1);
        }
    }
}
