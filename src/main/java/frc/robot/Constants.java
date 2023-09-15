// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    public static final class armConstants {
        public static final int Pitch1ID = 11; // CAN ID of one of the motors that controls the pitch of the arm

        public static final int ExtendID = 10; // CAN ID of the motor that extends and retracts arm

        public static final int intake1ID = 16;
        public static final int intake2ID = 17;

        public static final double maxExtent = 1;
        public static final double armLength = 1; //length (in meters) of the arm

        public static final double pitchOffset = -30;



        public static final Boolean Pitch1Invert = false;
        public static final Boolean Pitch2Invert = false;
        
        public static final Boolean ExtendInvert = true;
    }

    public static final class swerveModConstants {
        public static final double wheelP = 3;
        public static final double wheelI = 0;
        public static final double wheelD = 0;

        public static final double driveP = 0.5;
        public static final double driveI = 0;
        public static final double driveD = 0;


        public static final double kCPR = 1;


        // Front Right module
        public static final class mod1 {
            public static final int turningMotorID = 1; // CAN ID of the motor used for turning
            public static final int drivingMotorID = 5; // CAN ID of the motor used for driving
            public static final int encoderID = 0; // Analogue ID of the encoder used for this module (STARTS AT 0)
            public static final double encoderOffset = 37.001949; // Offset of the encoder (in degrees)
            public static final Boolean inverted = true; // Inverts the turning motor (change if the motor just accelerates in one direction)
            public static final Boolean driveInverted = false; // Inverts the drive motor (change if the wheel spins backwards)
        }

        // Back Right module
        public static final class mod2 {
            public static final int turningMotorID = 2;
            public static final int drivingMotorID = 6;
            public static final int encoderID = 1;
            public static final double encoderOffset = 190.107402;
            public static final Boolean inverted = true;
            public static final Boolean driveInverted = false;
        }

        // Back Left module
        public static final class mod3 {
            public static final int turningMotorID = 3;
            public static final int drivingMotorID = 7;
            public static final int encoderID = 2;
            public static final double encoderOffset = 219.902321;
            public static final Boolean inverted = true;
            public static final Boolean driveInverted = true;
        }

        // Front Left module
        public static final class mod4 {
            public static final int turningMotorID = 4;
            public static final int drivingMotorID = 8;
            public static final int encoderID = 3;
            public static final double encoderOffset = 282.832002;
            public static final Boolean inverted = true;
            public static final Boolean driveInverted = true;
        }

    public static final class driveConstants {

        // All values in meters
        
        public static final Boolean calibrate = false; // calibration mode for finding encoder offsets after a change in encoders

        public static final double kWheelBase = 0.0508; // The wheel base of the robot (Distance between front and back wheels)
        public static final double kTrack = 0.0635; // The track width of the robot (Side-to-side distance between wheels)

        public static final double kThetaMultiplier = -20; // Degrees/sec that the robot will turn when at full stick during teleop
        public static final double kSpeedMultiplier = 3.5; // Meters/sec that the robot will move when at full stick during teleop

        public static final double moduleRotateThreshold = 0.1; // The speed (in M/s) that a module will have to be instructed to move for the module to rotate

        public static final double kMaxSpeedMPS = 30; // Speed limiter of the robot, should be similar to kSpeedMultiplier
        public static final double kMaxAcceleration = 4; // Max acceleration of the robot (when 3 the robot will accelerate to full speed in 1/3 of a second)
    }

    public static final class autoConstants {
        public static final double lookAhaid = 0; //time that the robot should look ahaid on its path during auto

        public static final double lateralP = 1.8;
        public static final double lateralI = 0.001;
        public static final double lateralD = 0.1;

        public static final double rotationalP = 0.1;
        public static final double rotationalI = 0;
        public static final double rotationalD = 0;

        public static final String[] selectedPath = paths.taxi2;

        public static final class paths {
            public static final String[] taxi = {"forward", "backward", "taxi"};
            public static final String[] taxi2 = {"forward2", "backward2", "taxi2"};
        }
    }

    }
}
