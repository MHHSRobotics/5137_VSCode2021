package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ArcadeDrive extends Command {
	
	public ArcadeDrive() { 
		requires(Robot.driveBase_Subsystem);
	}

	protected void execute() { /*As you can see, whenever ArcadeDrive is scheduled to execute (which,
		due to being the default, is every 20ms), the code runs the method rampArcadeDrive() in the
		DriveBase subsystem. No other commands are here to potentially interrupt ArcadeDrive, so the
		isFinished() and interrupted() methods are irrelevant.
		
		The method has a variable inside its parentheses, which, as you should know by now since you're
		a pro, means the method has a parameter. We don't know what the parameter means just yet, but
		judging by the fact that the variable used here is the controller name jackBlack from the OI,
		I would predict that the parameter is the name of the controller that will be used. Let's jump
		back to the DriveBase to see what this method does.*/
		Robot.driveBase_Subsystem.rampArcadeDrive(Robot.oi.jackBlack);
	}

	// Called when another command which requires the driveBase is scheduled to run
	protected void interrupted() {
		end();
	}
	
	// Called when isFinished returns true (which never happens) or the command gets interrupted
	protected void end() {
		Robot.driveBase_Subsystem.stop();
	}

	protected boolean isFinished() {
		return false;
	}
	
}
