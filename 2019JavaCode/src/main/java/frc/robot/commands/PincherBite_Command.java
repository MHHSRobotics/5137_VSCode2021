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

public class PincherBite_Command extends Command {
  public PincherBite_Command() { /*Within the constructor of each command, there is the option to dec-
    lare what subsystem the command requires/belongs to. We must refer to the variable within Robot.java
    in order to declare the subsystem. This is that "one additional purpose" that you probably don't
    even remember me talking about at this point.*/

    requires(Robot.pincher_Subsystem);
  }

  boolean isFinished = false;

  /*This method is called one time, right before the command starts executing. We don't need to do any-
  thing here.*/
  @Override
  protected void initialize() {
  }

  /*This method is called periodically whenever the command is scheduled to execute. Let's see what
  happens.*/
  @Override
  protected void execute() {
    /*We start with an if statement, which uses the value of the piston to see whether it's in the
    off, open, or closed position. The code calls the getBitePincherStatus() method in the pincher sub-
    system, then tests to see if the value is NOT kReverse (!= means not equal to). If the value is
    NOT kReverse, the piston either hasn't been used yet or it is in the open position, so we want to
    close the pincher.*/
    if (Pincher_Subsystem.getBitePincherStatus() != DoubleSolenoid.Value.kReverse) { //CHANGES NEEDED
      Pincher_Subsystem.closePincher();
    }
    /*If getBitePincherStatus() does return kReverse, it means the piston is in the closed position.
    This means we want to open it instead.*/
    else {
      Pincher_Subsystem.openPincher();
    }
    /*After the command is executed, we set a variable "isFinished" to true.*/
    isFinished = true;
  }


  /*This method determines when the command is said to have finished executing. As long as this method
  returns false, the command will not be considered finished. Once it returns true, the command is con-
  sidered finished. The command may also finish depending on what type of button interaction the com-
  mand uses (if, for instance, the command is a whileHeld, the command will finish as soon as you re-
  lease the button). Since this command is a whenPressed type, there is no specified time as to when the
  command finishes executing, so we must manually tell it to finish using this method.
  
  The method returns "isFinished", the variable we created earlier. You'll notice that the command al-
  ways sets isFinished to true as soon as it starts executing. This means the command will return true
  and finish executing after only one 20ms time period.*/
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  /*end() is called one time after isFinished() returns true.*/
  @Override
  protected void end() {
  }

  /*interrupted() is that exception that you also forgot about when I talked about the scheduler. If
  you try to execute two commands at once, but both commands are part of the same subsystem, the first
  command's interrupted() method gets called. This runs whatever is inside the interrupted() method,
  and then removes the command from the scheduler. The point of this is to prevent two conflicting
  commands from running at once--for instance, you wouldn't want the robot to try to lift itself and
  lower itself at the same time, would you? That would make no sense.*/
  @Override
  protected void interrupted() {
    end();
  }
} /*You're making great progress with the robot code. All the other subsystems and commands are pretty
much the same, so we won't look at them--except for one more. The DriveBase_Subsystem works in generally
the same way, but is filled much more to the brim with content, so it's best that you look over it all
and understand it.*/
