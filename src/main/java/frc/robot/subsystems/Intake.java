package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team696.lib.HardwareDevices.TalonFactory;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private TalonFXConfiguration intakeMotorConfig;
    private TalonFactory intakeMotor;
    private TalonFXConfiguration intakeBackMotorConfig;
    private TalonFactory intakeBackMotor;
    private double velocity = 0.0;
    private MotionMagicVelocityTorqueCurrentFOC intakeControl;
    private MotionMagicVelocityTorqueCurrentFOC intakeControl2;
    private DigitalInput beamBreakSensor;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getIntakeSpeed() {
        return intakeMotor.getVelocity();
    }

    public void setIntakeSpeed(double intakeSpeed) {
        this.setVelocity(intakeSpeed);
    }

    private Intake() {
        intakeMotorConfig = new TalonFXConfiguration();
        intakeMotorConfig.withMotorOutput(
                new com.ctre.phoenix6.configs.MotorOutputConfigs()
                        .withInverted(Constants.Intake.frontInverted));
        intakeMotorConfig.withMotionMagic(
                new MotionMagicConfigs()
                        .withMotionMagicAcceleration(1000)
                        .withMotionMagicCruiseVelocity(1000)
                        .withMotionMagicJerk(0));
        intakeMotorConfig.withSlot0(
                new Slot0Configs()
                        .withKP(Constants.Intake.intakeKP)
                        .withKI(Constants.Intake.intakeKI)
                        .withKD(Constants.Intake.intakeKD));
        intakeMotor = new TalonFactory(Constants.Intake.frontIntakeID, intakeMotorConfig, "Intake Front Motor");
        intakeControl = new MotionMagicVelocityTorqueCurrentFOC(velocity, 0.0, true, 0.0, 0, false, false, false);
        intakeMotor.setControl(intakeControl);

        intakeBackMotorConfig = new TalonFXConfiguration();
        intakeBackMotorConfig.withMotorOutput(
                new com.ctre.phoenix6.configs.MotorOutputConfigs()
                        .withInverted(Constants.Intake.backInverted));
        intakeBackMotorConfig.withMotionMagic(
                new MotionMagicConfigs()
                        .withMotionMagicAcceleration(1000)
                        .withMotionMagicCruiseVelocity(1000)
                        .withMotionMagicJerk(0));
        intakeBackMotorConfig.withSlot0(
                new Slot0Configs()
                        .withKP(Constants.Intake.intakeKP)
                        .withKI(Constants.Intake.intakeKI)
                        .withKD(Constants.Intake.intakeKD));
        intakeBackMotor = new TalonFactory(Constants.Intake.backIntakeID, intakeBackMotorConfig, "Intake Back Motor");
        intakeControl2 = new MotionMagicVelocityTorqueCurrentFOC(velocity, 0.0, true, 0.0, 0, false, false, false);
        intakeBackMotor.setControl(intakeControl2);

        beamBreakSensor = new DigitalInput(0);
    }

    public void stop() {
        velocity = 0.0;
    }

    public void intake() {
        velocity = Constants.Intake.intakeVelocity;
    }

    public void outtakeVelocity() {
        velocity = Constants.Intake.outtakeVelocity;
    }

    public boolean getBeamBreak() {
        return beamBreakSensor.get();
    }

    @Override
    public void periodic() {
        intakeControl.Velocity = velocity;
        intakeControl2.Velocity = velocity;
        intakeMotor.setControl(intakeControl);
        intakeBackMotor.setControl(intakeControl2);
    }
}