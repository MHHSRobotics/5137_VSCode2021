
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AutoDriveStraight_Command extends CommandBase { //drives robot Forward (positive speed) or backward (negative speed) for a certain time

    double m_time;
    double m_speed;

    Timer m_timer;
    
    public AutoDriveStraight_Command(double time, double speed) {
        m_timer = new Timer();
        m_time = time;
        m_speed = speed;
        addRequirements(RobotContainer.driveBase_Subsystem);
    }

    //Called once when the command is scheduled to run
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        RobotContainer.driveBase_Subsystem.driveStraight(m_speed);
	}
	
	// Called when isFinished returns true (which never happens) or the command gets interrupted
	protected void end() {
        RobotContainer.driveBase_Subsystem.stop();
	}

    public boolean isFinished() {
        if (m_timer.get() < m_time) {
            return false;
        }
        else {
            return true;
        }
    } 
}