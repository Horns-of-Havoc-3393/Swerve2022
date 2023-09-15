package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armConstants;

public class ArmSubsystem extends SubsystemBase {

    // // private CANSparkMax Pitch;
    // // private RelativeEncoder pitchEncoder;

    // // private CANSparkMax Extend;
    // // private RelativeEncoder extendEncoder;

    // private TalonSRX intake1;
    // private TalonSRX intake2;

    // private AnalogInput pitchAnalog;

    // private Double pitch;
    // private Double length;

    // private DoublePublisher pitchPub;
    // private DoublePublisher lengthPub;
    // private DoublePublisher lengthRawPub;
    // private BooleanPublisher limitPub;

    // private AHRS IMU = new AHRS(SPI.Port.kMXP);

    // private Compressor compressor;
    // private DoubleSolenoid clawSolenoid1;
    // private DoubleSolenoid clawSolenoid2;

    // private DigitalInput limitSwitch;
    // private Boolean homing = true;
    // private Double home=0.0;

    // private SlewRateLimiter turnLimit = new SlewRateLimiter(3);

    // public ArmSubsystem() {
    //     NetworkTableInstance inst = NetworkTableInstance.getDefault();
    //     NetworkTable table = inst.getTable("/debug/arm");
    //     pitchPub = table.getDoubleTopic("pitch").publish();
    //     lengthPub = table.getDoubleTopic("length").publish();
    //     lengthRawPub = table.getDoubleTopic("lengthRaw").publish();
    //     limitPub = table.getBooleanTopic("limitSwitch").publish();

    //     compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    //     clawSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 0);
    //     clawSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

    //     limitSwitch = new DigitalInput(0);

    //     compressor.enableDigital();

    //     pitchAnalog = new AnalogInput(4);
        
    //     Pitch = new CANSparkMax(armConstants.Pitch1ID, MotorType.kBrushless);

    //     Extend = new CANSparkMax(armConstants.ExtendID, MotorType.kBrushless);

    //     intake1 = new TalonSRX(armConstants.intake1ID);
    //     intake2 = new TalonSRX(armConstants.intake2ID);


    //     Pitch.setInverted(armConstants.Pitch1Invert);
    //     Extend.setInverted(armConstants.ExtendInvert);
    //     extendEncoder = Extend.getEncoder();
    //     pitchEncoder = Pitch.getEncoder();

    //     homing = true;
    // }

    // public void runHome(){
    //     // if(homing){
    //     //     if(!limitSwitch.get()){
    //     //         Extend.set(-0.4);
    //     //     }else{
    //     //         Extend.set(0);
    //     //         homing = false;
    //     //         extendEncoder.setPosition(0.0);
    //     //     }
    //     // }
    //     return;
    // }

    // public void setIntakeSpeed(Double speed){
    //     intake1.set(TalonSRXControlMode.PercentOutput, -speed);
    //     intake2.set(TalonSRXControlMode.PercentOutput, speed);
    // }

    // public void sol1(Boolean extend){
    //     if(extend){
    //         clawSolenoid1.set(Value.kForward);
    //     }else{
    //         clawSolenoid1.set(Value.kReverse);
    //     }
    // }

    // public void sol2(Boolean extend){
    //     if(extend){
    //         clawSolenoid2.set(Value.kForward);
    //     }else{
    //         clawSolenoid2.set(Value.kReverse);
    //     }
    // }

    // public void grab(){
    //     clawSolenoid1.set(Value.kForward);
    //     clawSolenoid2.set(Value.kForward);
    // }

    // public void retract(){
    //     clawSolenoid1.set(Value.kReverse);
    //     clawSolenoid2.set(Value.kReverse);
    // }

    // public void relax(){
    //     clawSolenoid1.set(Value.kOff);
    //     clawSolenoid2.set(Value.kOff);
    // }

    // private double constrainArm(Double speed){
    //     double output = speed;
    //     if(getArmPitch().getDegrees()<=-20){
    //         output = Math.min(0, speed);
    //     }
    //     if(getArmPitch().getDegrees()>=70){
    //         output = Math.max(0,speed);
    //     }
    //     return output;
    // }


    // public void turnArm(double speed) {
    //     speed = turnLimit.calculate(speed-(1-Math.abs(getArmPitch().getDegrees()/90))*0);
    //     speed = constrainArm(speed);
    //     Pitch.set(speed);
    // }

    // public void extendArm(double speed){
    //     if(!homing){
    //         Extend.set(-0.4);
    //     }
    //     Extend.set(-0.4);
    // }

    // // public Boolean[] getConstraints(double turnSpeed, double extendSpeed){

    // // }

    // public Rotation2d getArmPitch(){
    //     double voltage = pitchAnalog.getVoltage();
    //     double angled = voltage*72;
    //     Rotation2d angle = Rotation2d.fromDegrees(angled*1.25*-1);
    //     angle = angle.minus(Rotation2d.fromDegrees(armConstants.pitchOffset));

    //     pitchPub.set(angle.getDegrees());
    //     return angle;
    // }

    // public double getArmLen(){
    // //     lengthRawPub.set((extendEncoder.getPosition()));
    // //     length = (extendEncoder.getPosition()/armConstants.maxExtent);
    // //     lengthPub.set(length);
    // //     return length;
    // // }

    // public void stop(){
    //     // turnArm(0);
    //     // if(!homing){
    //     //     Extend.set(0);
    //     // }
    // }

    // @Override
    // public void periodic() {
    //     runHome();
    //     limitPub.set(limitSwitch.get());
    //     Extend.set(-0.25);  
    //     System.out.print(home);
    //     getArmPitch();
    //     getArmLen();
    // }
}