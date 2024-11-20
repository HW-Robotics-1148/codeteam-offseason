package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutonomousShootCommand extends Command {
        private Rotation2d angle = new Rotation2d();
    private double velocityMPS = 0;
    public AutonomousShootCommand(){
        this.angle = angle;
        this.velocityMPS = velocityMPS;
        addRequirements(Shooter.getInstance(), Hood.getInstance(), Intake.getInstance());
    }


    @Override
    public void initialize() {
        Hood.getInstance().setAngle(angle);
        Shooter.getInstance().setVelocityMPS(velocityMPS);
    }

    @Override
    public void execute() {
        if(Shooter.getInstance().isAtVelocity() && Hood.getInstance().isAtAngle()){
            Intake.getInstance().intake();
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        Hood.getInstance().goHome();
        Shooter.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        // it was like Intake.getInstance.something i think beambreak??
        return false;
    }
    
}
