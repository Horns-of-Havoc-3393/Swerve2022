// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveModConstants;
import frc.robot.Constants.swerveModConstants.driveConstants;

public class SwerveModSubsystem extends SubsystemBase {

  private final PIDController turningPID;

  //private final turningMotor;
  private final TalonFX driveMotor;

  private final TalonSRX turningMotor;

  private final AnalogInput encoder;

  private final int encID;
  private final Rotation2d offsetR;

  private DoublePublisher rotationPub;
  private DoublePublisher voltagePub;
  private DoublePublisher targetAnglePub;
  private DoublePublisher targetVelPub;
  private DoublePublisher driveVelPub;
  private DoublePublisher driveVelTargetPub;

  private DoubleEntry turnPSub;
  private DoubleEntry turnISub;
  private DoubleEntry turnDSub;


  
  public SwerveModSubsystem(int drivingID, int turningID, int encID, Boolean inverted, Boolean driveInverted, double encOffset) {


    // initializes the NetworkTable publishers for remote debugging
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("/debug/mod" + (encID+1));
    rotationPub = table.getDoubleTopic("rotation").publish();
    voltagePub = table.getDoubleTopic("encVoltage").publish();
    targetAnglePub = table.getDoubleTopic("targetAngle").publish();
    targetVelPub = table.getDoubleTopic("targetVelocity").publish();
    driveVelPub = table.getDoubleTopic("driveWheelVelocity").publish();
    driveVelTargetPub = table.getDoubleTopic("driveWheelVelTarget").publish();

    turnPSub = inst.getDoubleTopic("/debug/turningP").getEntry(swerveModConstants.wheelP);
    turnISub = inst.getDoubleTopic("/debug/turningI").getEntry(swerveModConstants.wheelI);
    turnDSub = inst.getDoubleTopic("/debug/turningD").getEntry(swerveModConstants.wheelD);

    turnPSub.set(swerveModConstants.wheelP);
    turnISub.set(swerveModConstants.wheelI);
    turnDSub.set(swerveModConstants.wheelD);




    // Configures the drive motors (the motors that spin the wheels)
    driveMotor = new TalonFX(drivingID);
    driveMotor.setInverted(driveInverted);
    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.slot0.kF = 0.04690051; // Add 0.05 V output to overcome static friction
    config.slot0.kP = 0.01; // An error of 1 rps results in 0.11 V output
    config.slot0.kI = 0; // An error of 1 rps increases output by 0.5 V each second
    config.slot0.kD = 0.46; // An acceleration of 1 rps/s results in 0.001 V output

    driveMotor.configAllSettings(config);




    // configures the turning motor, its encoder, and its corresponding PID
    turningMotor = new TalonSRX(turningID);
    turningMotor.setInverted(inverted);

    this.encID = encID;
    offsetR = Rotation2d.fromDegrees(encOffset);

    encoder = new AnalogInput(encID);

    turningPID = new PIDController(swerveModConstants.wheelP,swerveModConstants.wheelI, swerveModConstants.wheelD);
    turningPID.enableContinuousInput(-Math.PI, Math.PI);

  }

  public void updatePID() {
    turningPID.setPID(turnPSub.get(), turnISub.get(), turnDSub.get());
  }

  private Rotation2d getAngle() {
    //double angle = relativeEncoder.getDistance();
    double voltage = encoder.getVoltage();

    voltagePub.set(voltage);

    double angle = ((voltage)*72);

    Rotation2d angleR = Rotation2d.fromDegrees(angle);

    angleR = angleR.minus(offsetR);

    if (angleR.getDegrees() < 0) {
      angleR = Rotation2d.fromDegrees(angleR.getDegrees()+360);
    }

    angleR = Rotation2d.fromDegrees(360-angleR.getDegrees());
    return angleR;
  }


  private double getWheelDistance() {
    return convertDriveSpeed(driveMotor.getSelectedSensorPosition(), false, false);
  } 

  private double convertDriveSpeed(double input, Boolean toDrive, Boolean speed) {

    double output;
    if(toDrive){
      output = input/((4*0.0254)*Math.PI);
      output = output*2048;
      if(speed) {output = output/10;}
    }else{
      output = input/2048;
      output = output*((4*2.54/100)*Math.PI);
      if(speed) {output = output*10;}
    }
    return output;
  }


  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(getWheelDistance(), getAngle());
  }


  public void setState(SwerveModuleState targetState) {
    targetState = SwerveModuleState.optimize(targetState, getAngle());

    driveMotor.set(TalonFXControlMode.Velocity, convertDriveSpeed(targetState.speedMetersPerSecond, true, true));
    turningMotor.set(TalonSRXControlMode.PercentOutput, turningPID.calculate(getAngle().getRadians(), targetState.angle.getRadians()));
    
    targetAnglePub.set(targetState.angle.getDegrees());
    targetVelPub.set(targetState.speedMetersPerSecond);

    driveVelTargetPub.set(driveMotor.getMotorOutputPercent());
    driveVelPub.set(driveMotor.getSelectedSensorVelocity());


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

    rotationPub.set(getAngle().getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
