package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class OnIntake_Command extends CommandBase {

    public OnIntake_Command() {
        addRequirements(RobotContainer.intake_Subsystem);
    }

    //Called when the command is initially scheduled
    @Override
    public void initialize() {
    }

    //Called every time the scheduler runs while the command is scheduled
    @Override
    public void execute() {
        RobotContainer.intake_Subsystem.toggleIntake(true, false);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.intake_Subsystem.toggleIntake(true, false);
    }

    @Override
    public boolean isFinished() {
        return true; //maybe change to false to keep going like drivebase 
    }
}