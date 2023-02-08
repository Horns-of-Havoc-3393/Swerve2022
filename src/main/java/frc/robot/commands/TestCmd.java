package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.swerveModConstants.driveConstants;
import frc.robot.subsystems.SwerveModSubsystem;
import frc.robot.subsystems.SwerveBaseSubsystem;

public class TestCmd extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private SwerveBaseSubsystem swerveBaseSubsystem;

    private final Translation2d m_frontLeftLoc;
    private final Translation2d m_backLeftLoc;
    private final Translation2d m_frontRightLoc;
    private final Translation2d m_backRightLoc;
  
    private final SwerveDriveKinematics kinematics;

    public TestCmd(SwerveBaseSubsystem swerveBaseSubsystem) {
        m_frontLeftLoc = new Translation2d(-driveConstants.kTrack/2, driveConstants.kWheelBase/2);
        m_backLeftLoc = new Translation2d(driveConstants.kTrack/-2, driveConstants.kWheelBase/2);
        m_frontRightLoc = new Translation2d(driveConstants.kTrack/2, driveConstants.kWheelBase/-2);
        m_backRightLoc = new Translation2d(driveConstants.kTrack/-2, driveConstants.kWheelBase/-2);

        kinematics = new SwerveDriveKinematics(
        m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc, m_backRightLoc
    );
        this.swerveBaseSubsystem = swerveBaseSubsystem;
        addRequirements(swerveBaseSubsystem);
    }

    @Override
    public void execute() {
        double xAxis = RobotContainer.driveController.getRawAxis(0);
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45));
        SwerveModuleState[] modStates = kinematics.toSwerveModuleStates(speeds);
        //System.out.print("\n" + modStates[0].angle.getDegrees() + " | " + modStates[1].angle.getDegrees() + " | " + modStates[2].angle.getDegrees() + " | " + modStates[3].angle.getDegrees());
        swerveBaseSubsystem.setSwerveState(modStates);
       
    }
}