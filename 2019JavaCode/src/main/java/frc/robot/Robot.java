/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*Robot.java is where all the action begins. When compiling and deploying (starting up) the code, this
class will be activated first. All the code is read one line at a time, from left to right, and then
from top to bottom.

The package in this case is simply the folder system in which the file exists. Imports are mainly to
bring in elements from written code in other places (such as WPI's library of stuff that they created
for Robotics) but they are also necessary if the code refers to other classes in the same code.*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot; /*Let's use this import as an example. This Robot.java class
is a subclass of TimedRobot, which means it has the properties of a TimedRobot class. (There are other
types of robot code such as Basic or Iterative, don't worry too much about them.) Since the code refers
to WPIlib's TimedRobot, TimedRobot must be imported. The great thing about VSCode is that it will 
suggest things to fix your code, one of which is "Import ___". Click that and it should automatically 
import something the code doesn't recognize.*/
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;

import frc.robot.subsystems.CargoBox_Subsystem; /*Here's another example of an import. This time it is
one which refers to another part of our own code.*/
import frc.robot.subsystems.DriveBase_Subsystem;
import frc.robot.subsystems.Lift_Subsystem;
import frc.robot.subsystems.Pincher_Subsystem;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends TimedRobot { /*After all the imports, we get to the actual class. Inside the
  class is everything that defines what it is and what it does. "What it is" = variables; "what it
  does" = methods. We've declared all the global variables here first. Variables have a number of
  purposes: they can denote objects that the robot will use or numerical values that we will use as
  data.

  These 4 variables will eventually go on to create our 4 subsystems. They have one additional pur-
  pose which will come into play later.*/
  public static DriveBase_Subsystem driveBase_Subsystem; 
  public static Pincher_Subsystem pincher_Subsystem;	
  public static CargoBox_Subsystem cargoBox_Subsystem;
  public static Lift_Subsystem lift_Subsystem;

  public static OI oi; /*The OI must be created after all other subsystems. It's something to do with
  the way FRC's code works and how it changes as it does certain things.*/

  /*After the subsystems, we begin creating our objects. These are our cameras; we'll get to them soon.*/
  public static UsbCamera frontCamera;
  public static UsbCamera leftCamera;
  public static UsbCamera rightCamera;

  //CHANGES NEEDED
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  /*This is our ultrasonic stuff. An ultrasonic (more commonly called a rangefinder or distance finder)
  is a device that measures distance by sending out a signal and measuring the time it takes for the
  signal to bounce back. Essentially we have a miniature dolphin on our robot.*/
  public static Ultrasonic ultrasonic1;
  public static Ultrasonic ultrasonic2;
  public static double distanceIN1;
  public static double distanceIN2;

  @Override
  public void robotInit() { /*robotInit() is a method. Methods are the "what it does" part of a robot--
    everything inside the method's curly brackets is read and executed whenever the method is called
    (told by the code to do its thing). robotInit() is called first whenever the code is compiled.

    The first thing robotInit() does is call RobotMap.init. This means it calls the method init() from
    RobotMap. Let's head over to RobotMap.java right now to see what it's doing.*/
    RobotMap.init();

    /*robotInit() is the place where we instantiate all our subsystems and the OI using constructors.
    Speaking of the OI, let's visit OI.java now and see what's going on in there.*/
    driveBase_Subsystem = new DriveBase_Subsystem();
	  pincher_Subsystem = new Pincher_Subsystem();
    cargoBox_Subsystem = new CargoBox_Subsystem();
    lift_Subsystem = new Lift_Subsystem();
		oi = new OI(); // gotta go after all the subsystems!


    // frontCamera = CameraServer.getInstance().startAutomaticCapture();
		// frontCamera.setResolution(320, 240);
    // frontCamera.setFPS(30); 
    
    // leftCamera = CameraServer.getInstance().startAutomaticCapture();
		// leftCamera.setResolution(320, 240);
    // leftCamera.setFPS(30); 
    
    // rightCamera = CameraServer.getInstance().startAutomaticCapture();
		// rightCamera.setResolution(320, 240);
		// rightCamera.setFPS(30); 


    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    ultrasonic1 = new Ultrasonic(6,7);
    ultrasonic1.setAutomaticMode(true); /*Turns on automatic mode. I don't really know what it is; all
    I do know is that it's far better than doing it manually.*/
    ultrasonic2 = new Ultrasonic(8,9); //CHANGES NEEDED
    ultrasonic2.setAutomaticMode(true);

  }

  @Override
  public void robotPeriodic() { /*Goodness me, all this time and I still haven't gotten into what peri-
    odic mode is!
  
    Anything referred to as "periodic" means that it will be executed every 20 milliseconds, which is 50
    times per second. It's quite useful to have something execute periodically if a value needs minute
    changes over time, such as when quickly adjusting your speed. Every 20ms the speed value will change
    depending on what the controller is inputting. Or you may want to read the distance from the ultra-
    sonic periodically and adjust depending on what the distance is. Bottom line: when you see "peri-
    odic", you think "every 20ms".
    
    This method, robotPeriodic(), runs the code inside the brackets every 20ms, regardless of what mode
    the robot is in. There are other modes, such as Autonomous and Tele-Op, that run only when the robot
    is in that particular mode. Take a look at your Driver Station and click the uppermost tab on the
    left, the one that looks like a steering wheel. This tab allows you to change the mode of your robot
    to Tele-Op, Autonomous, etc.*/
    
    distanceIN1 = ultrasonic1.getRangeInches();
    distanceIN2 = ultrasonic2.getRangeInches();

    /*CHANGES NEEDED for explanation of all camera things*/
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    
    if (x >= 1.0) {
      System.out.println(x);
    }
    if (y >= 1.0) {
      System.out.println(y);
    }
    if (area >= 1.0) {
      System.out.println(area);
    }


  }
  
  @Override
  public void autonomousInit() { /*This method is called one time when entering Autonomous mode, in
    which the robot drives itself without any input from human drivers. We won't be using Autonomous
    because FRC had the bright idea of making autonomous completely unnecessary in the year that our
    team used this code. Good job, FRC.*/

    m_autoSelected = m_chooser.getSelected();
    //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() { /*This method is called periodically during Autonomous mode. I have
    no idea what any of this stuff is, but I think it's something to do with being able to choose dif-
    ferent preset options for the autonomous code. As I said earlier, we didn't do anything with auto-
    nomous, so you may want to look into FRC's website for further clarification.*/

    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /*This method is called periodically during Tele-Op mode. Tele-Op is the mode in which you actually
  control the robot yourself.
  
  The Scheduler is in charge of scheduling commands and putting them into execution. If you press a
  button, the scheduler will put the corresponding command in queue until the command finishes execut-
  ing. Once execution finishes, the command is removed from the scheduler. If two buttons are pressed,
  the scheduler puts both of them into queue and then runs each of them alternately every 20ms. (Say
  for instance you hold the buttons to lift the robot and open the pincher at the same time. The sched-
  uler will execute the lift command during the first 20ms, then the pincher command during the next
  20ms, then the lift command again during the next 20ms, so on and so forth until the pincher command
  finishes executing and gets removed. From that point on, the lift command will be the only one in
  queue and run every 20ms.) There is a notable exception to this, which we'll see later.*/ 

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();	
  }

  /*This method is called periodically during test mode. We won't be using that mode either.*/
  @Override
  public void testPeriodic() {
  }
} /*Congratulations! You made it all the way through Main, Robot, RobotMap, and OI. These 4 files are
the backbone of the robot code, if you will. Now let's see some actual code in play. Let's visit the
Pincher_Subsystem to see what a subsystem looks like.*/
