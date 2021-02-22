package frc.robot.subsystems;

import frc.robot.commands.ArcadeDrive;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class DriveBase_Subsystem extends Subsystem { /*Welcome to the DriveBase subsystem, where all the
	driving happens It may seem a bit messy at first, but if you take a closer look at it, it actually
	follows the same pattern as the other subsystems. There is just a lot more stuff to look at.
	
	This section is simply an importation of stuff from RobotMap into here for our own use, as well
	as defining some informational values that we'll be using.*/
	Spark leftDriveMotor = RobotMap.leftDriveMotor;
	Spark rightDriveMotor = RobotMap.rightDriveMotor;
	Encoder leftEncoder = RobotMap.leftEncoder;
	Encoder rightEncoder = RobotMap.rightEncoder;
	DifferentialDriveOdometry odometry;
	Gyro gyro = new ADXRS450_Gyro(); 
	DifferentialDrive hotWheels = RobotMap.hotWheels;
	private double previousDriveSpeed = 0;
	public static double driveSpeed = 0; // for DisplayValues

	public DriveBase_Subsystem() {
		leftEncoder.setDistancePerPulse(.1173);
    	rightEncoder.setDistancePerPulse(.1173);

    	resetEncoders();
    	m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
	}

	@Override
  	public void periodic() {
    // Update the odometry in the periodic block
    	m_odometry.update(
        gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  	}


	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
	}

	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		odometry.resetPosition(pose, gyro.getRotation2d());
	}

	protected void initDefaultCommand() { /*We have a default command this time: ArcadeDrive. Arcade
		Drive is a form of driving in which you use one analog stick to control forwards and backwards
		movement, and turn left and right with the other stick. We want this to be running 24/7, so
		we will use this command and this command alone to drive the robot. No other commands are wired
		to the DriveBase subsystem so as to prevent it from getting interrupted and ending.
		
		We should take a look at ArcadeDrive to understand what it's doing.*/
		setDefaultCommand(new ArcadeDrive());
	}
	
	/*This algorithm, created by the fantastic Sarah C. Lincoln, creates a deadzone for the analog
	sticks. What does that mean exactly? Well, the controller measures the position of the sticks to an
	incredibly accurate degree--perhaps a little too accurate. We want a range of values for the sticks
	which will be ignored by the drive code; otherwise, the robot will constantly be trying to make
	tiny movements based on the sticks being in positions between, say, 0.2 and -0.2. This formula
	takes values that are close to 0 and simply makes them 0.
	
	This may be the first time you've seen a method in our code that has stuff inside its parentheses.
	These are simply parameters for our method. Remember how you can hover over methods to see their
	parameters? Well, we can create methods with parameters too! Essentially, parameters are like vari-
	ables in a function, like x and y. Whenever we call this method elsewhere in our code, we declare
	the values of x and y and then the formula does the math. In this particular case we declare what
	joystick we're using (jackBlack or redDead) and what our deadzone will be (+ or - 0.2, for example).
	
	As it stands, the robot never actually uses this code for anything because neither the ArcadeDrive
	command nor any other methods in the subsystem actually call this method. Why did I give it an
	explanation, then? Well, it certainly wasn't that I just didn't realize that fact until later and
	was too proud of my work to get rid of it...that would be just silly, wouldn't it? Ha ha...*/
	public double adjustJoystickValue(double joystick, double deadZone) {
		double adjustedJoystick;
		if (Math.abs(joystick) < deadZone) {
			adjustedJoystick = 0;
		} else {
			adjustedJoystick = ((1 / (1 - deadZone)) * (joystick - deadZone));
		}
		return adjustedJoystick;
	}
	
	/*Here's the method that ArcadeDrive calls. As you can see, the method establishes a parameter
	with the properties of a Joystick, so we were right about our prediction that the parameter
	selects the joystick we want to use. Right then, so what's going on?*/
	public void rampArcadeDrive(Joystick jackBlack) {
		/*Our first course of action is to get the values of the analog sticks from the controller.
		Our second course of action is to use the distance variables from the Ultrasonic code we
		created all the way back in Robot.java (revisit it if you like).*/
		double driveJoystick = jackBlack.getRawAxis(1);
		double turnJoystick = jackBlack.getRawAxis(4);
		double distanceIN1 = Robot.distanceIN1;
		double distanceIN2 = Robot.distanceIN2;
		
		/*After that, we create a variable newDriveSpeed and set it to a value. What value is it?
		That is determined by the method accelerate(). We'll need to jump down to that method in
		order to determine what we're getting here.*/
		double newDriveSpeed;
		newDriveSpeed = accelerate(driveJoystick, previousDriveSpeed, .4, .05);
		driveSpeed = newDriveSpeed; // to print to SmartDashboard
		previousDriveSpeed = newDriveSpeed;
		if (distanceIN1 > 6.0 || driveJoystick >= 0.1) {
			hotWheels.arcadeDrive(newDriveSpeed, -turnJoystick);
		}
		
	}

	public double accelerate(double joystickValue, double previousSpeed, double minSpeed,
	double incValue) {
		int delay = 25;
		double newSpeed;
		
		// effectively a deadzone with range +-minValue
		if (Math.abs(joystickValue) > minSpeed && Math.abs(previousSpeed) < minSpeed) {
			newSpeed = Math.signum(joystickValue) * minSpeed;
		} else {
			newSpeed = previousSpeed;
		}
		
		if (previousSpeed < joystickValue) newSpeed += incValue;
		else if (previousSpeed > joystickValue) newSpeed -= incValue;
		else newSpeed = previousSpeed; // necessary??
		try {
			Thread.sleep(delay);
		} catch (InterruptedException e) {

		}
		
		return newSpeed;
	}

	public void driveStraight(double speed) {
		hotWheels.arcadeDrive(-speed, 0);
	}
	
	// CW is positive, CCW is negative
	public void pivot(double speed) {
		hotWheels.arcadeDrive(0, -speed); 
	}
	
	public void stop() {
		hotWheels.arcadeDrive(0, 0);
	}



		//public void arcadeDrive(Joystick jackBlack) {
	   /*
        * x' = a(x^b) + (1-a)x
        *
        * x' is the scaled output value
        * a is the vertical/horizontal stretch
        * x is the raw value input from the axis
        * as a approaches 0, the ramp approaches a linear (y=x) ramp
        * as a approaches 1 the ramp approaches an exponential ramp
        * increasing the b makes the ramp's severity lower
        * (it takes more to move the stick for it to reach its full value)
        * (looks like a x^b)
        * decreasing the b between 1 and 0 makes the ramp's severity higher
        * (looks like a root x )
        * don't make a negative. that is NOT GOOD
        */
        
		/*
        double rawArcade = jackBlack.getRawAxis(1);
        double rawSlide = jackBlack.getRawAxis(0);
        double rawTurn = jackBlack.getRawAxis(4);
        */
        
        // dead zone included
      /*  double rawDrive = adjustJoystickValue(jackBlack.getRawAxis(1), .2);
        double rawTurn = adjustJoystickValue(jackBlack.getRawAxis(4), .2); 
        
        double sensitivityDrive = .95;
        double sensitivityTurn = .95;
        
        double exponentialDrive;
        double exponentialTurn;
        
        double sensitivityExponent = 3;
        
        exponentialDrive = (sensitivityDrive*(Math.pow(rawDrive, sensitivityExponent))) + ((1-sensitivityDrive)*rawDrive);
        exponentialTurn = -1*(sensitivityTurn*(Math.pow(rawTurn, sensitivityExponent))) + ((1-sensitivityTurn)*rawTurn);
        
        hotWheels.arcadeDrive(exponentialDrive, exponentialTurn);
	}
	*/

}
