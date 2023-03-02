package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveModConstants.driveConstants;

import java.lang.reflect.Field;




public class PositioningSubsystem extends SubsystemBase {

    DoublePublisher xPub;
    DoublePublisher yPub;

    SwerveDriveOdometry odometry;

    
    AHRS IMU = new AHRS(SPI.Port.kMXP);

    private final Translation2d frontLeftLoc;
    private final Translation2d backLeftLoc;
    private final Translation2d frontRightLoc;
    private final Translation2d backRightLoc;

    private final SwerveDriveKinematics swerveKinematics;

    private Pose2d odoPose;
    private Field2d field;

    private SwerveBaseSubsystem swerveBaseSubsystem;

    public PositioningSubsystem() {

        
        frontLeftLoc = new Translation2d(-driveConstants.kTrack/2, driveConstants.kWheelBase/2);
        backLeftLoc = new Translation2d(-driveConstants.kTrack/2, -driveConstants.kWheelBase/2);
        frontRightLoc = new Translation2d(driveConstants.kTrack/2, driveConstants.kWheelBase/2);
        backRightLoc = new Translation2d(driveConstants.kTrack/2, -driveConstants.kWheelBase/2);
        swerveKinematics = new SwerveDriveKinematics(
            frontLeftLoc, frontRightLoc, backLeftLoc, backRightLoc
        );

        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("/debug/position");
        field = new Field2d();
        xPub = table.getDoubleTopic("xPosition").publish();
        yPub = table.getDoubleTopic("yPosition").publish();
        
        IMU.calibrate();

        new Thread(()-> {
            try{
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e){}
        }).start();
    }

    public void initOdometry(SwerveBaseSubsystem swerveBaseSubsystem){
        this.swerveBaseSubsystem = swerveBaseSubsystem;
        odometry = new SwerveDriveOdometry(swerveKinematics, IMU.getRotation2d(), swerveBaseSubsystem.getSwervePositions());
    }

    @Override
    public void periodic() {
        // do odometry
        odoPose = odometry.update(IMU.getRotation2d(), swerveBaseSubsystem.getSwervePositions());
        field.setRobotPose(odoPose);
        SmartDashboard.putData("Field", field);
    }


    public Pose2d getPose() {
        return odoPose;
    }



    public Rotation3d getQuaternion() {
        Rotation3d output = new Rotation3d(
            IMU.getQuaternionX()/2*Math.PI, IMU.getQuaternionY()/2*Math.PI, IMU.getQuaternionZ()/2*Math.PI
        );
        return output;
    }

    public double getRotationVelocity() {
        return IMU.getRate();
    }


    public void zeroHeading(){
        IMU.reset();
    }

}
