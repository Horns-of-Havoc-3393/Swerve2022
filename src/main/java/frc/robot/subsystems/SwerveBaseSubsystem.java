package frc.robot.subsystems;

import javax.swing.text.Position;

import org.opencv.core.RotatedRect;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.swerveModConstants;
import frc.robot.Constants.swerveModConstants.autoConstants;
import frc.robot.Constants.swerveModConstants.driveConstants;

public class SwerveBaseSubsystem extends SubsystemBase {

    private final SwerveModSubsystem frontLeftMod = new SwerveModSubsystem(
        swerveModConstants.mod4.drivingMotorID, swerveModConstants.mod4.turningMotorID, swerveModConstants.mod4.encoderID,
        swerveModConstants.mod4.inverted, swerveModConstants.mod4.driveInverted, swerveModConstants.mod4.encoderOffset
    );

    private final SwerveModSubsystem backLeftMod = new SwerveModSubsystem(
        swerveModConstants.mod3.drivingMotorID, swerveModConstants.mod3.turningMotorID, swerveModConstants.mod3.encoderID,
        swerveModConstants.mod3.inverted, swerveModConstants.mod3.driveInverted, swerveModConstants.mod3.encoderOffset
    );

    private final SwerveModSubsystem frontRightMod = new SwerveModSubsystem(
        swerveModConstants.mod1.drivingMotorID, swerveModConstants.mod1.turningMotorID, swerveModConstants.mod1.encoderID,
        swerveModConstants.mod1.inverted, swerveModConstants.mod1.driveInverted, swerveModConstants.mod1.encoderOffset
    );

    private final SwerveModSubsystem backRightMod = new SwerveModSubsystem(
        swerveModConstants.mod2.drivingMotorID, swerveModConstants.mod2.turningMotorID, swerveModConstants.mod2.encoderID,
        swerveModConstants.mod2.inverted, swerveModConstants.mod2.driveInverted, swerveModConstants.mod2.encoderOffset
    );


    DoublePublisher anglePub;
    DoublePublisher ratePub;
    DoublePublisher constantPub;

    DoublePublisher xPub;
    DoublePublisher yPub;
    DoublePublisher thetaPub;

    private final Translation2d frontLeftLoc;
    private final Translation2d backLeftLoc;
    private final Translation2d frontRightLoc;
    private final Translation2d backRightLoc;

    private final SwerveDriveKinematics swerveKinematics;

    private final PIDController thetaPID;


    private PositioningSubsystem positioningSubsystem;




    // Constants relating to acceleration limit
    private SwerveModuleState[] accelerationRefStates;
    private double accelerationRefTime;
    private Boolean accelerationLimitOn;




    public SwerveBaseSubsystem(PositioningSubsystem positioningSubsystem) {

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("/debug/robot");
        anglePub = table.getDoubleTopic("gyroAngle").publish();
        ratePub = table.getDoubleTopic("gyroRate").publish();
        constantPub = table.getDoubleTopic("thetaConversionConstant").publish();

        xPub = table.getDoubleTopic("foX").publish();
        yPub = table.getDoubleTopic("foY").publish();
        thetaPub = table.getDoubleTopic("foTheta").publish();

        frontLeftLoc = new Translation2d(-driveConstants.kTrack/2, driveConstants.kWheelBase/2);
        backLeftLoc = new Translation2d(-driveConstants.kTrack/2, -driveConstants.kWheelBase/2);
        frontRightLoc = new Translation2d(driveConstants.kTrack/2, driveConstants.kWheelBase/2);
        backRightLoc = new Translation2d(driveConstants.kTrack/2, -driveConstants.kWheelBase/2);

        thetaPID = new PIDController(driveConstants.thetaP, driveConstants.thetaI, driveConstants.thetaD);
        thetaPID.setTolerance(driveConstants.thetaTolerance);

        this.positioningSubsystem = positioningSubsystem;
        positioningSubsystem.initOdometry(this);


        //this.initAccelerationLimit();


        swerveKinematics = new SwerveDriveKinematics(
            frontLeftLoc, frontRightLoc, backLeftLoc, backRightLoc
        );

        

    }

    public void updateTurnPIDs(){
        frontLeftMod.updatePID();
        frontRightMod.updatePID();
        backLeftMod.updatePID();
        backRightMod.updatePID();
    }

    

    public Rotation2d getBaseAngle() {
        double rawAngle = positioningSubsystem.getQuaternion().getZ();
        Rotation2d angle = new Rotation2d(rawAngle);
        anglePub.set(angle.getDegrees());
        return angle;
    }

    public double getBaseRate() {
        double rate = positioningSubsystem.getRotationVelocity();
        ratePub.set(rate);
        return rate;
    }

    public SwerveModulePosition[] getSwervePositions() {
        SwerveModulePosition[] output = {
            frontLeftMod.getModulePosition(),
            frontRightMod.getModulePosition(),
            backLeftMod.getModulePosition(),
            backRightMod.getModulePosition()
        };
        return output;
    }

    public void toPoint(Pose2d targetPoint) {
        setFieldOriented(targetPoint.minus(positioningSubsystem.getPose()).getX()*autoConstants.lookAhaid,
            targetPoint.minus(positioningSubsystem.getPose()).getY()*autoConstants.lookAhaid,
            targetPoint.minus(positioningSubsystem.getPose()).getRotation().getDegrees()*autoConstants.lookAhaid
        );
    }

    public void setFieldOriented(double xAxis, double yAxis, double thetaAxis) {
        double theta = thetaAxis*driveConstants.kThetaMultiplier;

        yPub.set(yAxis*driveConstants.kSpeedMultiplier);
        xPub.set(xAxis*driveConstants.kSpeedMultiplier);
        thetaPub.set(thetaAxis*driveConstants.kThetaMultiplier);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xAxis*driveConstants.kSpeedMultiplier, yAxis*driveConstants.kSpeedMultiplier, 
            theta,
            //theta+thetaPID.calculate(this.getBaseRate()-theta, 0),
            this.getBaseAngle()
        );

        constantPub.set(Math.log(theta)/Math.log(this.getBaseRate()));

        SwerveModuleState[] targetStates = swerveKinematics.toSwerveModuleStates(speeds);
        this.setSwerveState(targetStates);
    }

    public void setRelative(double xAxis, double yAxis, double thetaAxis) {
        double theta = thetaAxis*driveConstants.kThetaMultiplier;

        yPub.set(yAxis*driveConstants.kSpeedMultiplier);
        xPub.set(xAxis*driveConstants.kSpeedMultiplier);
        thetaPub.set(thetaAxis*driveConstants.kThetaMultiplier);

        ChassisSpeeds speeds = new ChassisSpeeds(yAxis, xAxis, thetaAxis);

        SwerveModuleState[] targetStates = swerveKinematics.toSwerveModuleStates(speeds);
        this.setSwerveState(targetStates);
    }



    // public void initAccelerationLimit() {
    //     SwerveModuleState zeroState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0));
    //     for(int i=0; i<4; i++) {
    //         accelerationRefStates[i] = zeroState;
    //     }

    //     accelerationRefTime = System.currentTimeMillis();
    //     accelerationLimitOn = true;
    // }

    // public void disableAccelerationLimit() {
    //     accelerationLimitOn = false;
    // }

    // private double getCurrentVelLimit() {
    //     return 0.0;
    // }

    public void setSwerveState(SwerveModuleState[] targetStates) {

        
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, driveConstants.kMaxSpeedMPS);

        double[] swerveTargets = {(targetStates[0].angle.getDegrees()),(targetStates[1].angle.getDegrees()),(targetStates[2].angle.getDegrees()),(targetStates[3].angle.getDegrees())};
        SmartDashboard.putNumberArray("swerveTargets", swerveTargets);
        frontLeftMod.setState(targetStates[0]);
        backLeftMod.setState(targetStates[1]);
        frontRightMod.setState(targetStates[2]);
        backRightMod.setState(targetStates[3]);
    }
}