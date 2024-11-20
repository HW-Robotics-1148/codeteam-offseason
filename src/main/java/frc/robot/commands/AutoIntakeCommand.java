package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntakeCommand extends Command {
    @Override
    public void initialize() {
        Intake.getInstance().intake();
        addRequirements(Intake.getInstance());
    }
    @Override
    public void execute() {
        Intake.getInstance().intake();
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return !Intake.getInstance().getBeamBreak();
    }

}
