package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveModConstants;
import frc.robot.Constants.swerveModConstants.autoConstants;
import frc.robot.Constants.swerveModConstants.driveConstants;

public class SwerveBaseSubsystem extends SubsystemBase {

    // Initializes the subsystems for each swerve module
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

    DoublePublisher newXPub;
    DoublePublisher newYPub;
    DoublePublisher xAxisPub;
    DoublePublisher yAxisPub;
    DoublePublisher distancePub;
    DoublePublisher targetAngPub;
    DoubleEntry lateralPPub;
    DoubleEntry lateralIPub;
    DoubleEntry lateralDPub;
    DoubleEntry rotationalPPub;
    DoubleEntry rotationalIPub;
    DoubleEntry rotationalDPub;

    BooleanSubscriber redSub;

    private final Translation2d frontLeftLoc;
    private final Translation2d backLeftLoc;
    private final Translation2d frontRightLoc;
    private final Translation2d backRightLoc;

    private final SwerveDriveKinematics swerveKinematics;



    private PositioningSubsystem positioningSubsystem;

    private PIDController lateralPID1;
    private PIDController lateralPID2;
    private PIDController rotationalPID;




    // Constants relating to acceleration limit
    private SlewRateLimiter xLimit = new SlewRateLimiter(driveConstants.kMaxAcceleration);
    private SlewRateLimiter yLimit = new SlewRateLimiter(driveConstants.kMaxAcceleration);
    private SlewRateLimiter rLimit = new SlewRateLimiter(3);




    public SwerveBaseSubsystem(PositioningSubsystem positioningSubsystem) {

        // NetworkTable init for telemetry
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("/debug/robot");
        anglePub = table.getDoubleTopic("gyroAngle").publish();
        ratePub = table.getDoubleTopic("gyroRate").publish();
        constantPub = table.getDoubleTopic("thetaConversionConstant").publish();

        xPub = table.getDoubleTopic("foX").publish();
        yPub = table.getDoubleTopic("foY").publish();

        NetworkTable auto = inst.getTable("/debug/auto");
        newXPub = auto.getDoubleTopic("NewX").publish();
        newYPub = auto.getDoubleTopic("NewY").publish();
        xAxisPub = auto.getDoubleTopic("xAxis").publish();
        yAxisPub = auto.getDoubleTopic("yAxis").publish();
        distancePub = auto.getDoubleTopic("distance").publish();
        targetAngPub = auto.getDoubleTopic("targetAngle").publish();
        lateralPPub = auto.getDoubleTopic("lateralP").getEntry(autoConstants.lateralP);
        lateralIPub = auto.getDoubleTopic("lateralI").getEntry(autoConstants.lateralI);
        lateralDPub = auto.getDoubleTopic("lateralD").getEntry(autoConstants.lateralD);
        rotationalPPub = auto.getDoubleTopic("rotationalP").getEntry(autoConstants.rotationalP);
        rotationalIPub = auto.getDoubleTopic("rotationalI").getEntry(autoConstants.rotationalI);
        rotationalDPub = auto.getDoubleTopic("rotationalD").getEntry(autoConstants.rotationalD);

        lateralPPub.set(autoConstants.lateralP);
        lateralIPub.set(autoConstants.lateralI);
        lateralDPub.set(autoConstants.lateralD);
        rotationalPPub.set(autoConstants.rotationalP);
        rotationalIPub.set(autoConstants.rotationalI);
        rotationalDPub.set(autoConstants.rotationalD);

        thetaPub = table.getDoubleTopic("foTheta").publish();

        NetworkTable FMSInfo = inst.getTable("/FMSInfo");
        redSub = FMSInfo.getBooleanTopic("IsRedAlliance").subscribe(false);

        // Locations of swerve modules
        frontLeftLoc = new Translation2d(-driveConstants.kTrack/2, driveConstants.kWheelBase/2);
        backLeftLoc = new Translation2d(-driveConstants.kTrack/2, -driveConstants.kWheelBase/2);
        frontRightLoc = new Translation2d(driveConstants.kTrack/2, driveConstants.kWheelBase/2);
        backRightLoc = new Translation2d(driveConstants.kTrack/2, -driveConstants.kWheelBase/2);


        // Init positioning subsystem
        this.positioningSubsystem = positioningSubsystem;
        positioningSubsystem.initOdometry(this);


        //this.initAccelerationLimit();


        swerveKinematics = new SwerveDriveKinematics(
            frontLeftLoc, frontRightLoc, backLeftLoc, backRightLoc
        );


        // PID controllers
        lateralPID1 = new PIDController(lateralPPub.get(), lateralIPub.get(), lateralDPub.get());
        lateralPID2 = new PIDController(lateralPPub.get(), lateralIPub.get(), lateralDPub.get());
        rotationalPID = new PIDController(rotationalPPub.get(), rotationalIPub.get(), rotationalDPub.get());
        rotationalPID.enableContinuousInput(-Math.PI, Math.PI);

        

    }


    // Updates the cooefficients of the PID values
    public void updateTurnPIDs(){
        frontLeftMod.updatePID();
        frontRightMod.updatePID();
        backLeftMod.updatePID();
        backRightMod.updatePID();

        lateralPID1.setPID(lateralPPub.get(), lateralIPub.get(), lateralDPub.get());
        lateralPID2.setPID(lateralPPub.get(), lateralIPub.get(), lateralDPub.get());
        rotationalPID.setPID(rotationalPPub.get(), rotationalIPub.get(), rotationalDPub.get());
    }

    
    // Returns yaw angle of the robot
    private Rotation2d getBaseAngle() {
        double rawAngle = positioningSubsystem.getYawAngle();
        Rotation2d angle = Rotation2d.fromDegrees(rawAngle);
        anglePub.set(angle.getDegrees());
        return angle;
    }

    // Returns yaw rate of the robot
    private double getBaseRate() {
        double rate = positioningSubsystem.getRotationVelocity();
        ratePub.set(rate);
        return rate;
    }

    // Gathers the positions of all of the swerve modules
    public SwerveModulePosition[] getSwervePositions() {
        SwerveModulePosition[] output = {
            frontLeftMod.getModulePosition(),
            backLeftMod.getModulePosition(),
            frontRightMod.getModulePosition(),
            backRightMod.getModulePosition()
        };
        return output;
    }

    // Moves the robot towards a poit in absolute space
    public void toPoint(Pose2d targetPoint) {
        double newTheta = targetPoint.getRotation().times(-1).minus(Rotation2d.fromDegrees(180)).minus(Rotation2d.fromDegrees(positioningSubsystem.getYawAngle())).getRadians();

        
        Pose2d currentPose = positioningSubsystem.getPose();
        
        double distanceX = targetPoint.getX()-currentPose.getX();
        double distanceY = targetPoint.getY()-currentPose.getY();

        distancePub.set(Math.sqrt((distanceX*distanceX)+(distanceY*distanceY)));
        targetAngPub.set(targetPoint.getRotation().getDegrees());

        if(redSub.get()){
            this.setFieldOriented(lateralPID1.calculate(currentPose.getX(), targetPoint.getX())*-1,
                lateralPID2.calculate(currentPose.getY(), targetPoint.getY()),
                rotationalPID.calculate(0, newTheta),
                true
            );
        }else{
            this.setFieldOriented(lateralPID1.calculate(currentPose.getX(), targetPoint.getX()),
                lateralPID2.calculate(currentPose.getY(), targetPoint.getY())*-1,
                rotationalPID.calculate(0, newTheta),
                true
            );
        }
    }

    // Calculates acceleration limits
    private double[] doAccelerationLimit(double xAxis, double yAxis, double thetaAxis){

        double[] output = {
            xLimit.calculate(xAxis),
            yLimit.calculate(yAxis),
            rLimit.calculate(thetaAxis)
        };
        return output;
    }

    // Resets the current position of odometry to a provided value
    public void setPosition(double x, double y){
        positioningSubsystem.setPosition(x, y);
    }

    // Sets the speed of the robot oriented to the field
    public void setFieldOriented(double xAxis, double yAxis, double thetaAxis, boolean absolute) {
        double theta = thetaAxis;
        if(absolute == false){

            // accleration limits and speed multipliers
            double[] axes = doAccelerationLimit(xAxis, yAxis, thetaAxis);
            xAxis = axes[0]*driveConstants.kSpeedMultiplier;
            yAxis = axes[1]*driveConstants.kSpeedMultiplier;
            thetaAxis = axes[2]*driveConstants.kThetaMultiplier;


            yPub.set(yAxis*driveConstants.kSpeedMultiplier);
            xPub.set(xAxis*driveConstants.kSpeedMultiplier);
            thetaPub.set(thetaAxis*driveConstants.kThetaMultiplier);
        }

        // Creates chassis speeds instance
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xAxis, yAxis, 
            thetaAxis,
            //theta+thetaPID.calculate(this.getBaseRate()-theta, 0),
            this.getBaseAngle()
        );

        constantPub.set(Math.log(theta)/Math.log(this.getBaseRate()));

        // Calculates swerve states and moves them
        SwerveModuleState[] targetStates = swerveKinematics.toSwerveModuleStates(speeds);
        this.setSwerveState(targetStates);
    }


    // non field-orented swerve control
    public void setRelative(double xAxis, double yAxis, double thetaAxis) {

        // acceleration limits and speed multipliers
        double[] axes = doAccelerationLimit(xAxis, yAxis, thetaAxis);
        xAxis = axes[0];
        yAxis = axes[1];
        thetaAxis = axes[2];

        yPub.set(yAxis*driveConstants.kSpeedMultiplier);
        xPub.set(xAxis*driveConstants.kSpeedMultiplier);
        thetaPub.set(thetaAxis*driveConstants.kThetaMultiplier);


        // Directly sends relative chassis speeds to the swerve modules
        ChassisSpeeds speeds;

        if(!redSub.get()){
            speeds = new ChassisSpeeds(xAxis, yAxis, thetaAxis);
        }else{
            speeds = new ChassisSpeeds(-xAxis, -yAxis, thetaAxis);
        }

        SwerveModuleState[] targetStates = swerveKinematics.toSwerveModuleStates(speeds);
        this.setSwerveState(targetStates);
    }


    // Applies swerve module states to the swerve modules
    public void setSwerveState(SwerveModuleState[] targetStates) {

        // makes sure no wheel is moving faster than the max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, driveConstants.kMaxSpeedMPS);

        // sends swerve stuff through telemetry
        double[] swerveTargets = {(targetStates[0].angle.getDegrees()),
            (targetStates[1].angle.getDegrees()),
            (targetStates[2].angle.getDegrees()),
            (targetStates[3].angle.getDegrees())};
        SmartDashboard.putNumberArray("swerveTargets", swerveTargets);

        // Sets the states of all the individual modules
        frontLeftMod.setState(targetStates[0]);
        backLeftMod.setState(targetStates[1]);
        frontRightMod.setState(targetStates[2]);
        backRightMod.setState(targetStates[3]);
    }
}