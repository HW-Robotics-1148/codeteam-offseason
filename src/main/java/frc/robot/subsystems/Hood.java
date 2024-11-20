package frc.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team696.lib.HardwareDevices.TalonFactory;

public class Hood extends SubsystemBase {
    private final double ARM_ROTATIONS_TO_MOTOR_ROTATIONS = 24.796 / 0.25;
    private static Hood instance;
    private TalonFXConfiguration hoodLeftConfigurator;
    private TalonFactory hoodLeftMotor;
    private TalonFXConfiguration hoodRightConfigurator;
    private TalonFactory hoodRightMotor;
    private MotionMagicExpoVoltage hoodLeftControl;
    private MotionMagicExpoVoltage hoodRightControl;
    private Rotation2d armAngle = new Rotation2d();

    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood();
        }
        return instance;
    }

    public static void setInstance(Hood instance) {
        Hood.instance = instance;
    }

    public Rotation2d getAngleTar() {
        return armAngle;
    }

    public void setAngle(Rotation2d angle) {
        this.armAngle = angle;
    }

    public void goHome() {
        this.setAngle(new Rotation2d());
    }

    public boolean isAtAngle() {
        return Math.abs((hoodLeftMotor.getPosition() * (1 / ARM_ROTATIONS_TO_MOTOR_ROTATIONS)
                + hoodRightMotor.getPosition() * (1 / ARM_ROTATIONS_TO_MOTOR_ROTATIONS)) / 2
                - getAngleTar().getRotations()) <= 0.01;
    }

    public double getHoodSpeed() {
        return hoodLeftMotor.getVelocity();
    }

    public void setHoodAngle(Rotation2d hoodAngle) {
        this.setAngle(hoodAngle);
    }

    private Hood() {
        hoodLeftConfigurator = new TalonFXConfiguration();
        hoodLeftConfigurator.withMotorOutput(new MotorOutputConfigs().withInverted(Constants.Hood.leftInverted)
                .withNeutralMode(NeutralModeValue.Brake));
        this.hoodLeftControl = new MotionMagicExpoVoltage(
                armAngle.getRotations() * ARM_ROTATIONS_TO_MOTOR_ROTATIONS, true, 0.0, 0, false, false, false);
        hoodLeftConfigurator.withMotionMagic(
                new MotionMagicConfigs().withMotionMagicAcceleration(240)
                        .withMotionMagicCruiseVelocity(240));
        hoodLeftConfigurator.withSlot0(new Slot0Configs().withKP(Constants.Hood.hoodKP)
                .withKI(Constants.Hood.hoodKI).withKD(Constants.Hood.hoodKD).withKA(Constants.Hood.hoodKA)
                .withKV(Constants.Hood.hoodKV));
        hoodLeftConfigurator.withHardwareLimitSwitch(
                new HardwareLimitSwitchConfigs().withForwardLimitAutosetPositionValue(Constants.Hood.maxAngle)
                        .withReverseLimitAutosetPositionValue(Constants.Hood.minAngle));
        hoodLeftMotor = new TalonFactory(Constants.Hood.leftHoodID, hoodLeftConfigurator, "Hood Left Motor");
        hoodLeftMotor.setControl(hoodLeftControl);

        hoodRightConfigurator = new TalonFXConfiguration();
        hoodRightConfigurator.withMotorOutput(
                new MotorOutputConfigs().withInverted(Constants.Hood.rightInverted)
                        .withNeutralMode(NeutralModeValue.Brake));
        this.hoodRightControl = new MotionMagicExpoVoltage(
                armAngle.getRotations() * ARM_ROTATIONS_TO_MOTOR_ROTATIONS, true, 0.0, 0, false, false, false);
        hoodRightConfigurator.withMotionMagic(
                new MotionMagicConfigs().withMotionMagicAcceleration(240)
                        .withMotionMagicCruiseVelocity(240));
        hoodRightConfigurator.withSlot0(
                new Slot0Configs().withKP(Constants.Hood.hoodKP)
                        .withKI(Constants.Hood.hoodKI).withKD(Constants.Hood.hoodKD)
                        .withKA(Constants.Hood.hoodKA).withKV(Constants.Hood.hoodKV));
        hoodRightConfigurator.withHardwareLimitSwitch(
                new HardwareLimitSwitchConfigs().withForwardLimitAutosetPositionValue(Constants.Hood.maxAngle)
                        .withReverseLimitAutosetPositionValue(Constants.Hood.minAngle));
        hoodRightMotor = new TalonFactory(Constants.Hood.rightHoodID, hoodRightConfigurator, "Hood Right Motor");
        hoodRightMotor.setControl(hoodRightControl);

        this.setAngle(new Rotation2d(Math.PI / 2));

    }

    @Override
    public void periodic() {
        hoodLeftControl.Position = armAngle.getRotations() * ARM_ROTATIONS_TO_MOTOR_ROTATIONS;
        hoodRightControl.Position = armAngle.getRotations() * ARM_ROTATIONS_TO_MOTOR_ROTATIONS;
        hoodLeftMotor.setControl(
                hoodLeftControl.withSlot(0).withPosition(armAngle.getRotations() * ARM_ROTATIONS_TO_MOTOR_ROTATIONS));
        hoodRightMotor.setControl(
                hoodRightControl.withSlot(0).withPosition(armAngle.getRotations() * ARM_ROTATIONS_TO_MOTOR_ROTATIONS));
    }
}