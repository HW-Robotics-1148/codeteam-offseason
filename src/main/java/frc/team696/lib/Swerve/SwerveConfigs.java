package frc.team696.lib.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.team696.lib.Swerve.SwerveConstants;

public final class SwerveConfigs {
        public final static TalonFXConfiguration angle;
        public final static TalonFXConfiguration drive;
        public final static CANcoderConfiguration canCoder;
        public final static Pigeon2Configuration pigeon;
        public final static SwerveModuleConstants Mod0;
        public final static SwerveModuleConstants Mod1;
        public final static SwerveModuleConstants Mod2;
        public final static SwerveModuleConstants Mod3;

        static {
                angle = new TalonFXConfiguration();
                drive = new TalonFXConfiguration();
                canCoder = new CANcoderConfiguration();
                pigeon = new Pigeon2Configuration();

                Mod0 = new SwerveModuleConstants();
                Mod1 = new SwerveModuleConstants();
                Mod2 = new SwerveModuleConstants();
                Mod3 = new SwerveModuleConstants();

                /** Swerve CANCoder Configuration */
                canCoder.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
                canCoder.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

                /** Swerve Angle Motor Configuration */
                angle.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                angle.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                angle.Feedback.SensorToMechanismRatio = SwerveConstants.angleGearRatio; // Updated gear ratio
                angle.ClosedLoopGeneral.ContinuousWrap = true;
                angle.CurrentLimits.SupplyCurrentLimitEnable = true;
                angle.CurrentLimits.SupplyCurrentLimit = 25;
                angle.CurrentLimits.SupplyCurrentThreshold = 40;
                angle.CurrentLimits.SupplyTimeThreshold = 0.1;
                angle.CurrentLimits.StatorCurrentLimitEnable = true;
                angle.CurrentLimits.StatorCurrentLimit = 40;
                angle.Slot0.kP = 100.0; // Updated PID constant
                angle.Slot0.kI = 0.0; // Updated PID constant
                angle.Slot0.kD = 0.0; // Updated PID constant

                angle.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
                angle.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

                angle.Voltage.PeakForwardVoltage = 16.0;
                angle.Voltage.PeakReverseVoltage = -16.0;

                /** Swerve Drive Motor Configuration */
                drive.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                drive.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                drive.Feedback.SensorToMechanismRatio = SwerveConstants.driveGearRatio; // Updated gear ratio
                drive.CurrentLimits.SupplyCurrentLimitEnable = true;
                drive.CurrentLimits.SupplyCurrentLimit = 25;
                drive.CurrentLimits.SupplyCurrentThreshold = 60;
                drive.CurrentLimits.SupplyTimeThreshold = 0.2;
                drive.CurrentLimits.StatorCurrentLimitEnable = true;
                drive.CurrentLimits.StatorCurrentLimit = 60;

                drive.Voltage.PeakForwardVoltage = 16.0;
                drive.Voltage.PeakReverseVoltage = -16.0;

                drive.Slot0.kP = 2.0; // Updated PID constant
                drive.Slot0.kI = 0.0; // Updated PID constant
                drive.Slot0.kD = 0.0; // Updated PID constant
                drive.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2; // Updated ramp period
                drive.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2; // Updated ramp period
                drive.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02; // Updated ramp period
                drive.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02; // Updated ramp period

                Mod0.CANcoderId = 12;
                Mod0.DriveMotorId = 4;
                Mod0.SteerMotorId = 3;
                Mod0.CANcoderOffset = Rotation2d.fromRotations(-0.034 + 0.5).getRotations(); // Updated offset

                // Module 1 Configuration (Front Right)
                Mod1.CANcoderId = 13;
                Mod1.DriveMotorId = 6;
                Mod1.SteerMotorId = 5;
                Mod1.CANcoderOffset = Rotation2d.fromRotations(-0.456).getRotations(); // Updated offset

                // Module 2 Configuration (Back Left)
                Mod2.CANcoderId = 11;
                Mod2.DriveMotorId = 2;
                Mod2.SteerMotorId = 1;
                Mod2.CANcoderOffset = Rotation2d.fromRotations(-0.178 + 0.5).getRotations(); // Updated offset

                // Module 3 Configuration (Back Right)
                Mod3.CANcoderId = 14;
                Mod3.DriveMotorId = 8;
                Mod3.SteerMotorId = 7;
                Mod3.CANcoderOffset = Rotation2d.fromRotations(0.258).getRotations(); // Updated offset

                /** Pigeon Configuration */
                pigeon.MountPose.MountPoseYaw = 0.0; // Updated yaw angle
        }
}