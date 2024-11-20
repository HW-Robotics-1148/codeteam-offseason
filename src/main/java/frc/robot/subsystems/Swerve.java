package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Util;
import frc.team696.lib.Swerve.SwerveDriveSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SwerveDriveSubsystem {
    private static Swerve instance;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    private NeutralModeValue neutralMode = Constants.Swerve.driveNeutralMode;
    private SendableChooser<NeutralModeValue> neutralModeChooser = new SendableChooser<>();
    // private PoseEstimator poseEstimator;

    public NeutralModeValue getNeutralMode() {
        return neutralMode;
    }

    public double getDistToSpeaker() {
        if (Util.getAlliance() == Alliance.Red)
            return distTo(Constants.Field.RED.Speaker);

        return distTo(Constants.Field.BLUE.Speaker);
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        this.neutralMode = neutralMode;
    }

    public SwerveModule[] getmSwerveMods() {
        return mSwerveMods;
    }

    public Pigeon2 gyro;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "drive");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        // poseEstimator = null;// PoseEstimator.getInstance();

        neutralModeChooser.setDefaultOption("Brake", NeutralModeValue.Brake);
        neutralModeChooser.addOption("Coast", NeutralModeValue.Coast);

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getGyroYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        // Sets center of rotation to front of robot for sick drifts
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void fromChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    /* Used by SwerveControllerCommand in Auto */

    public void stop() {
        setVelocity(0);
    }

    public void setVelocity(double value) {
        SwerveModuleState[] states = new SwerveModuleState[getModuleCount()];
        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveModuleState(value, new Rotation2d());
        }
        setModuleStates(states);
    }

    public int getModuleCount() {
        return mSwerveMods.length;
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getRobotVelocity() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(this.getModuleStates());
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public double getMeasurement() {
        double sum = 0;
        for (SwerveModule mod : mSwerveMods) {
            sum += mod.getState().speedMetersPerSecond;
        }
        return sum / mSwerveMods.length;
    }

    double distance = 0.0;

    public void onUpdate() {
        System.out.println(distance);
        distance += getRobotVelocity().vyMetersPerSecond * 0.02;
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        // Smart Dashboard numbers
        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
        SmartDashboard.putNumber("Gyro Yaw", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("Robot Velocity", getRobotVelocity().vxMetersPerSecond);
        SmartDashboard.putNumber("Robot Angular Velocity", getRobotVelocity().omegaRadiansPerSecond);
        SmartDashboard.putData(neutralModeChooser);

        // Allow operator to change neutral mode on the fly
        if (neutralMode != neutralModeChooser.getSelected()) {
            setNeutralMode(neutralModeChooser.getSelected());
        }
    }

}