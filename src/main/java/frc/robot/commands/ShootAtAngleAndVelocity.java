package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class ShootAtAngleAndVelocity extends Command {
    private Rotation2d angle = new Rotation2d();
    
    private double velocityMPS = 0;
    public ShootAtAngleAndVelocity(Rotation2d angle, double velocityMPS){
        this.angle = angle;
        this.velocityMPS = velocityMPS;
        addRequirements(Shooter.getInstance(), Hood.getInstance());
    }


    @Override
    public void initialize() {
        Hood.getInstance().setAngle(angle);
        Shooter.getInstance().setVelocityMPS(velocityMPS);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        Hood.getInstance().goHome();
        Shooter.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
