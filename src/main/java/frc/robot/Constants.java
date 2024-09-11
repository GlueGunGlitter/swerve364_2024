package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.util.COTSTalonFXSwerveConstants;

import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public class AmpAssistConstants {

        public static final double SPEED_WHILE_DRIVE_TO_AMP_WHENE_SEE_APRILTAG = 0.6;
        public static final double SPEED_WHILE_DRIVE_TO_AMP_WHENE_NOT_SEE_APRILTAG = 0.3;
        public static final double TIME_DELAY_BETWEEN_SHOOTER_AND_TRANSPORTATION = 1;
        public static final double METERS_OF_START_SHOTER = 2.5;
        public static final double KP = 0.5;
        public static final double KD = 0.05;

    }

    public class AlignWithAmpConstans {
        public static final double TOLERANCE_OF_DGREE = 4;
        public static final double KP = 0.0055;
        public static final double KD = 0.00055;

    }


    public class AimAssistConstans {
        public static final double SPEED_WHILE_DRIVE_TO_NOTE = 0.6;
        public static final double KP = 0.008;
        public static final double KD = 0.0008;
        public static final double TIME_DELAY_BETWEEN_STOPING_AIM_ASSIST_AND_STOP_TRANSPORTATION = 3;

        
    }

    public class ClimbConstants {

        public static final int CLIMB_RIGHT_MOTOR_PORT = 61;
        public static final int CLIMB_LEFT_MOTOR_PORT = 60;

        // the ports needs to be change

    }

    public class ShooterConstants {

        // the ports needs to be change
        public static final int NON_STATIC_MOTOR_PORT = 41;
        // Might be something else for the non static as it was basiclly imposiable to
        // understand what excatly was written on it
        public static final int STATIC_MOTOR_PORT = 42;
        public static final double MAX_SPEED_OF_NON_STATIC_MOTOR = 5000;
        public static final double MAX_SPEED_OF_STATIC_MOTOR = 5000;

        public static final double SHOOT_TIMEOUT = 1.5;
    }

    // public class ConveyorConstants {
    // public static final int CONVEYOR_TALON_PORT = 40;

    // }

    public static final class TransportationConstants {

        // the ports needs to be change
        public static final int INTAKE_LOWER_MOTOR_PORT = 22;
        public static final int INTAKE_HIGHER_MOTOR_PORT = 21;
        public static final int TRANSPORTATION_MOTOR_PORT_ONE = 31;
        public static final int TRANSPORTATION_MOTOR_PORT_TWO = 32;

    }

    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific
                                                                      // robot
                COTSTalonFXSwerveConstants.SDS.MK4i
                        .Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.775; // TODO: This must be tuned to specific robot
        public static final double wheelBase = 0.725; // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double robotRadius = 436.46;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 20;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 40;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 14;
            public static final int canCoderID = 51; // it was 3
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-23.55);// -20.83);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 53;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-169.36); // 49.85);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot

            public static final int driveMotorID = 3;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 52;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-129.90);; // 10.19);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 54;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(112.23);// 112.41);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3.5;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(10, 0.0, 1), // Translation PID constants
                new PIDConstants(2, 0.005, 0.2), // Rotation constants
                kMaxSpeedMetersPerSecond,
                Swerve.robotRadius / 1000, // Drive base radius (distance from center to furthest module)
                new ReplanningConfig());
    }
}
