package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 18;

        public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); // TODO: This must be tuned to specific
                                                                             // robot
        public static final double wheelBase = Units.inchesToMeters(21.73); // TODO: This must be tuned to specific
                                                                            // robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

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
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.02;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 2.0; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = (0.967 / 12); // TODO: This must be tuned to specific robot
        public static final double driveKV = (5.44 / 12);
        public static final double driveKA = (0.57 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.034 + 0.5);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.456);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.178 + 0.5);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.258);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class Intake {
        public static final int backIntakeID = 20;
        public static final int frontIntakeID = 21;

        public static final double intakeKP = 1.2;
        public static final double intakeKI = 0;
        public static final double intakeKD = 0;

        public static final InvertedValue frontInverted = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue backInverted = InvertedValue.Clockwise_Positive;

        public static final double intakeVelocity = 70;
        public static final double outtakeVelocity = -70;

    }

    public static final class Hood {
        public static final int leftHoodID = 26;
        public static final int rightHoodID = 24;

        public static final double hoodKP = 0.8;
        public static final double hoodKI = 0;
        public static final double hoodKD = 0.0;

        public static final double hoodKA = 0;
        public static final double hoodKV = 0;

        public static final InvertedValue leftInverted = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue rightInverted = InvertedValue.Clockwise_Positive;

        public static final double maxAngle = 30;
        public static final double minAngle = 0;
    }

    public static final class Shooter { // TODO: FIX CUS QUINN SUCKS
        public static final int leftShooterID = 22;
        public static final int rightShooterID = 23;

        public static final double shooterKP = 4.0;
        public static final double shooterKI = 0;
        public static final double shooterKD = 0;

        public static final InvertedValue leftInverted = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue rightInverted = InvertedValue.Clockwise_Positive;
    }

    public static final class Field {
        public static final class RED {
            public static final Translation2d Speaker = new Translation2d(16.57, 5.54);
            public static final Pose2d Amp = new Pose2d(14.7, 7.8, new Rotation2d(Math.PI / 2));
            public static final Pose2d Source = new Pose2d(1, 0.5, Rotation2d.fromDegrees(-135));
            public static final Translation2d Corner = new Translation2d(14.57, 7.);

        }

        public static final class BLUE {
            public static final Translation2d Speaker = new Translation2d(-0.04, 5.54);
            public static final Pose2d Amp = new Pose2d(1.7, 7.8, new Rotation2d(Math.PI / 2));
            public static final Pose2d Source = new Pose2d(15.15, 1.5, Rotation2d.fromDegrees(135));
            public static final Translation2d Corner = new Translation2d(2., 7.);

        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 4.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = 1.5 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 1.5 * Math.PI;

        public static final PathConstraints GLOBAL_CONSTRAINTS = new PathConstraints(kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared, kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);

        private static PIDConstants translationConstants = new PIDConstants(4.0, 0.0, 0.0);
        private static PIDConstants rotationConstants = new PIDConstants(4.0, 0, 0.0);
        private static ReplanningConfig replanningConfig = new ReplanningConfig();

        public static HolonomicPathFollowerConfig getPathFollowerConfig() {
            return new HolonomicPathFollowerConfig(translationConstants, rotationConstants,
                    Constants.Swerve.maxSpeed,
                    Constants.Swerve.trackWidth / 2.0, replanningConfig);
        }

    }
}
