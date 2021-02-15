/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Pincher_Subsystem extends Subsystem { /*Subsystems organize the robot by breaking down
  all of its functions into smaller sections. This pincher subsystem, for instance, contains all the
  commands that the pincher subsystem can use, such as opening, closing, extending, and retracting.

  We start by importing the pneumatic pistons we created in RobotMap into here. Now we can manipulate
  them to do what we want.*/
  public static DoubleSolenoid pincherBitePiston = RobotMap.pincherBitePiston;
  public static DoubleSolenoid pincherSlidePiston = RobotMap.pincherSlidePiston;

  @Override
  public void initDefaultCommand() { /*You have the option of giving the subsystem a default command.
    The default command will run periodically if no other commands in the subsystem are running. The
    command will stop once you call one of the commands in the subsystem, and once it stops executing,
    the default command starts running again. The syntax of adding a default command is as follows:
    
    setDefaultCommand(new _____());
    
    where you fill the blank with a constructor for one of your commands. If you don't establish a
    default command, the subsystem will do nothing when no commands are active.*/
  }

  /*The subsystem contains all of the methods that it can execute, but it doesn't actually say when
  they execute. That information is provided by the Command classes. When a command receives the signal
  from the OI to run, it calls a method within its subsystem. The subsystem then executes the method.
  
  This method and the one below it detect the position of the pneumatic piston. The piston's resting
  position is kOff, which is in the middle of the piston. Its other values are kForward, which is a
  complete extension in the forward direction, and kReverse, which is a complete extension in the rev-
  erse direction. We can use this information to determine whether the pincher is in the open or
  closed position, and by extension, whether we need to open or close the pincher.*/
  public static DoubleSolenoid.Value getBitePincherStatus() {
    System.out.println(pincherBitePiston.get());
    return pincherBitePiston.get();
  }

  public static DoubleSolenoid.Value getSlidePincherStatus() {
    System.out.println(pincherSlidePiston.get());
    return pincherSlidePiston.get();
  }

  /*The following two methods change the state of the pneumatic piston to either the open or closed
  state. The two methods afterwards do the same thing, except using a different piston.*/
  public static void openPincher() {
		pincherBitePiston.set(DoubleSolenoid.Value.kForward); //CHANGES NEEDED
	}
	
	public static void closePincher() {
		pincherBitePiston.set(DoubleSolenoid.Value.kReverse); //CHANGES NEEDED
	}
  
  public static void extendSlidePiston() {
		pincherSlidePiston.set(DoubleSolenoid.Value.kForward); //CHANGES NEEDED
	}
	
	public static void retractSlidePiston() {
		pincherSlidePiston.set(DoubleSolenoid.Value.kReverse); //CHANGES NEEDED
  }
  
  /*This method changes the state of both pneumatic positions to the off state.*/
  public void stop() {
    pincherBitePiston.set(DoubleSolenoid.Value.kOff);
    pincherSlidePiston.set(DoubleSolenoid.Value.kOff);
  }
} /*Now that we know what our subsystem does, we can look at a command within the subsystem. Let's
take a look at the PincherBite_Command.*/
