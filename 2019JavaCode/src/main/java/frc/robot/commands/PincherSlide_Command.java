/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Pincher_Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class PincherSlide_Command extends Command {
  public PincherSlide_Command() {
    requires(Robot.pincher_Subsystem);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  boolean isFinished = false;

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Pincher_Subsystem.getSlidePincherStatus() != DoubleSolenoid.Value.kReverse) { //CHANGES NEEDED
      System.out.println("Trying to close");
      Pincher_Subsystem.closePincher();
    }
    else {
      System.out.println("Trying to open");
      Pincher_Subsystem.openPincher();
    }
    isFinished = true;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.pincher_Subsystem.stop();
  }
}
