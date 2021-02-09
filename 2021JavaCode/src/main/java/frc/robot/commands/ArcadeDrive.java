// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ArcadeDrive extends CommandBase {

    public ArcadeDrive() {
        addRequirements(RobotContainer.driveBase_Subsystem);
    }

    //Called once when the command is scheduled to run
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        /* As you can see, whenever ArcadeDrive is scheduled to execute (which,
		due to being the default, is every 20ms), the code runs the method rampArcadeDrive() in the
		DriveBase subsystem. No other commands are here to potentially interrupt ArcadeDrive, so the
        isFinished() and end() methods are irrelevant.*/
        System.out.println("DriveBase is running...");
        RobotContainer.driveBase_Subsystem.rampArcadeDrive(RobotContainer.XBoxController);
	}
	
	// Called when isFinished returns true (which never happens) or the command gets interrupted
	protected void end() {
        RobotContainer.driveBase_Subsystem.stop();
	}

    public boolean isFinished() {
		return true;
    } 
}
