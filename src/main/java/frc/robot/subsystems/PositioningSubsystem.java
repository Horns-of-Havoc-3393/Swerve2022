package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveModConstants.driveConstants;




public class PositioningSubsystem extends SubsystemBase {

    DoublePublisher xPub;
    DoublePublisher yPub;

    BooleanSubscriber redSub;

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

        // NetworkTable init for Telemetry
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

        NetworkTable FMSInfo = inst.getTable("/FMSInfo");
        redSub = FMSInfo.getBooleanTopic("IsRedAlliance").subscribe(false);
        

        // IMU init stuff
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

    // Resets the saved odometry position depending on team
    public void setPosition(double x, double y){
        if(redSub.get()){
            odometry.resetPosition(Rotation2d.fromDegrees(180), swerveBaseSubsystem.getSwervePositions(), new Pose2d(new Translation2d(16.54-x,y), Rotation2d.fromDegrees(180)));
        }else{
            odometry.resetPosition(Rotation2d.fromDegrees(0), swerveBaseSubsystem.getSwervePositions(), new Pose2d(new Translation2d(x,y), Rotation2d.fromDegrees(0)));
        }
    }

    @Override
    public void periodic() {
        // do odometry
        odoPose = odometry.update(IMU.getRotation2d(), swerveBaseSubsystem.getSwervePositions());
        field.setRobotPose(odoPose);
        SmartDashboard.putData("Field", field);
    }


    // Returns the odometry pose
    public Pose2d getPose() {
        return odoPose;
    }


    // Gets quaternion from the IMU
    public Rotation3d getQuaternion() {
        Rotation3d output = new Rotation3d(
            IMU.getQuaternionX()/2*Math.PI, IMU.getQuaternionY()/2*Math.PI, IMU.getQuaternionZ()/2*Math.PI
        );
        return output;
    }

    public double getYawAngle() {
        return IMU.getAngle();
    }

    public double getRotationVelocity() {
        return IMU.getRate();
    }


    public void zeroHeading(){
        IMU.reset();
        if(redSub.get()){
            IMU.setAngleAdjustment(180);
        }else{
            IMU.setAngleAdjustment(0);
        }
    }

    // zeros the odometry position
    public void zeroPosition(Boolean update){
        if(update){
            odometry.resetPosition(IMU.getRotation2d(), swerveBaseSubsystem.getSwervePositions(), new Pose2d(new Translation2d(0,0), IMU.getRotation2d()));
        }
        if(!redSub.get()){
            odometry.resetPosition(Rotation2d.fromDegrees(0), swerveBaseSubsystem.getSwervePositions(), new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)));
        }else{
            odometry.resetPosition(Rotation2d.fromDegrees(180), swerveBaseSubsystem.getSwervePositions(), new Pose2d(new Translation2d(16.25,6.858), Rotation2d.fromDegrees(180)));
        }
    }

}
