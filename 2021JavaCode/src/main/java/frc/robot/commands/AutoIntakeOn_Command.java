package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoIntakeOn_Command extends CommandBase {

    Timer m_timer;
    double ourTime;

    public AutoIntakeOn_Command(double time) {
        m_timer = new Timer();
        ourTime = time;
        addRequirements(RobotContainer.intake_Subsystem);
    }

    //Called when the command is initially scheduled
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
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
        if (m_timer.get() < ourTime) {
            return false;
        }
        else {
             return true; 
        }
       
    }
}