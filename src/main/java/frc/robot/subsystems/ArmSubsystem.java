package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    private CANSparkMax Pitch;
    private RelativeEncoder pitchEncoder;

    private CANSparkMax Extend;
    private RelativeEncoder extendEncoder;

    private AnalogInput pitchAnalog;

    private Double pitch;
    private Double length;

    private DoublePublisher pitchPub;
    private DoublePublisher lengthPub;
    private DoublePublisher lengthRawPub;
    private BooleanPublisher limitPub;

    private AHRS IMU = new AHRS(SPI.Port.kMXP);

    private Compressor compressor;
    private DoubleSolenoid clawSolenoid1;
    private DoubleSolenoid clawSolenoid2;

    private DigitalInput limitSwitch;
    private Boolean homing = false;
    private Double home=0.0;

    private SlewRateLimiter turnLimit = new SlewRateLimiter(3);

    public ArmSubsystem() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("/debug/arm");
        pitchPub = table.getDoubleTopic("pitch").publish();
        lengthPub = table.getDoubleTopic("length").publish();
        lengthRawPub = table.getDoubleTopic("lengthRaw").publish();
        limitPub = table.getBooleanTopic("limitSwitch").publish();

        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        clawSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        clawSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

        limitSwitch = new DigitalInput(0);

        compressor.enableDigital();

        pitchAnalog = new AnalogInput(4);
        
        Pitch = new CANSparkMax(armConstants.Pitch1ID, MotorType.kBrushless);

        Extend = new CANSparkMax(armConstants.ExtendID, MotorType.kBrushless);


        Pitch.setInverted(armConstants.Pitch1Invert);
        Extend.setInverted(armConstants.ExtendInvert);
        extendEncoder = Extend.getEncoder();

        homing = false;
    }

    public void runHome(){
        if(homing){
            if(!limitSwitch.get()){
                Extend.set(TalonSRXControlMode.PercentOutput, -0.2);
            }else{
                Extend.set(TalonSRXControlMode.PercentOutput, 0);
                homing = false;
                home = Extend.getSelectedSensorPosition();
            }
        }
        return;
    }

    public void sol1(Boolean extend){
        if(extend){
            clawSolenoid1.set(Value.kForward);
        }else{
            clawSolenoid1.set(Value.kReverse);
        }
    }

    public void sol2(Boolean extend){
        if(extend){
            clawSolenoid2.set(Value.kForward);
        }else{
            clawSolenoid2.set(Value.kReverse);
        }
    }

    public void grab(){
        clawSolenoid1.set(Value.kForward);
        clawSolenoid2.set(Value.kForward);
    }

    public void retract(){
        clawSolenoid1.set(Value.kReverse);
        clawSolenoid2.set(Value.kReverse);
    }

    public void relax(){
        clawSolenoid1.set(Value.kOff);
        clawSolenoid2.set(Value.kOff);
    }


    public void turnArm(double speed) {
        speed = turnLimit.calculate(speed-(1-Math.abs(getArmPitch().getDegrees()/90))*0.4);
        Pitch1.set(TalonSRXControlMode.PercentOutput, speed);
        Pitch2.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void extendArm(double speed){
        if(!homing){
            Extend.set(TalonSRXControlMode.PercentOutput, speed);
        }
    }

    // public Boolean[] getConstraints(double turnSpeed, double extendSpeed){

    // }

    public Rotation2d getArmPitch(){
        double voltage = pitchAnalog.getVoltage();
        double angled = voltage*72;
        Rotation2d angle = Rotation2d.fromDegrees(angled*1.25*-1);
        angle = angle.minus(Rotation2d.fromDegrees(armConstants.pitchOffset));

        pitchPub.set(angle.getDegrees());
        return angle;
    }

    public double getArmLen(){
        Extend.set(TalonSRXControlMode.PercentOutput, -0.15);
        lengthRawPub.set((Extend.getSelectedSensorPosition()-home)/4096/6.867919921875);
        length = (Extend.getSelectedSensorPosition()-home)/4096/6.867919921875;
        lengthPub.set(length);
        return length;
    }

    public void stop(){
        turnArm(0);
        if(!homing){
            Extend.set(TalonSRXControlMode.PercentOutput, 0);
        }
    }

    @Override
    public void periodic() {
        runHome();
        limitPub.set(limitSwitch.get());
        Extend.set(TalonSRXControlMode.PercentOutput, -0.25);  
        System.out.print(home);
        getArmPitch();
        getArmLen();
    }
}