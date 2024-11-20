package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team696.lib.HardwareDevices.TalonFactory;

public class Shooter extends SubsystemBase {

    private static Shooter instance;
    private TalonFXConfiguration leftShootConfig;
    private TalonFactory leftShootMotor;
    private TalonFXConfiguration rightShootConfig;
    private TalonFactory rightShootMotor;
    private double velocity;
    private MotionMagicVelocityTorqueCurrentFOC leftShootControl;
    private MotionMagicVelocityTorqueCurrentFOC rightShootControl;
    private final double MPS_TO_RPS = 1;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public double getVelocityTarMPS() {
        return velocity;
    }

    public void setVelocityMPS(double velocity) {
        this.velocity = velocity;
    }

    public void stop() {
        this.setVelocityMPS(0);
    }

    public boolean isAtVelocity() {
        return Math.abs(
                (leftShootMotor.getVelocity() * (1 / MPS_TO_RPS)
                        + rightShootMotor.getVelocity() * (1 / MPS_TO_RPS)) / 2
                        - getVelocityTarMPS()) <= 0.25;
    }

    public Shooter() {
        leftShootConfig = new TalonFXConfiguration();
        leftShootConfig.withMotorOutput(
                new com.ctre.phoenix6.configs.MotorOutputConfigs()
                        .withInverted(Constants.Shooter.leftInverted));
        leftShootConfig.withMotionMagic(
                new MotionMagicConfigs()
                        .withMotionMagicAcceleration(1000)
                        .withMotionMagicCruiseVelocity(velocity)
                        .withMotionMagicJerk(0));
        leftShootConfig.withSlot0(
                new Slot0Configs()
                        .withKP(Constants.Shooter.shooterKP)
                        .withKI(Constants.Shooter.shooterKI)
                        .withKD(Constants.Shooter.shooterKD));
        leftShootMotor = new TalonFactory(Constants.Shooter.leftShooterID, leftShootConfig, "Left Shooter Motor");
        leftShootControl = new MotionMagicVelocityTorqueCurrentFOC(velocity, 0.0, true, 0.0, 0, false, false, false);
        leftShootMotor.setControl(leftShootControl);

        rightShootConfig = new TalonFXConfiguration();
        rightShootConfig.withMotorOutput(
                new com.ctre.phoenix6.configs.MotorOutputConfigs()
                        .withInverted(Constants.Shooter.rightInverted));
        rightShootConfig.withMotionMagic(
                new MotionMagicConfigs()
                        .withMotionMagicAcceleration(1000)
                        .withMotionMagicCruiseVelocity(velocity)
                        .withMotionMagicJerk(0));
        rightShootConfig.withSlot0(
                new Slot0Configs()
                        .withKP(Constants.Shooter.shooterKP)
                        .withKI(Constants.Shooter.shooterKI)
                        .withKD(Constants.Shooter.shooterKD));
        rightShootMotor = new TalonFactory(Constants.Shooter.rightShooterID, rightShootConfig, "Right Shooter Motor");
        rightShootControl = new MotionMagicVelocityTorqueCurrentFOC(velocity, 0.0, true, 0.0, 0, false, false, false);
        rightShootMotor.setControl(rightShootControl);
    }

    @Override
    public void periodic() {
        leftShootControl.Velocity = velocity * MPS_TO_RPS;
        rightShootControl.Velocity = velocity * MPS_TO_RPS;
        leftShootMotor.setControl(leftShootControl);
        rightShootMotor.setControl(rightShootControl);
    }
}