package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class RobotMap { /*RobotMap is a class that's set aside to create all the physical objects on the
	robot. This includes motors, encoders (devices that measure how far a wheel has turned to deter-
	mine the distance the wheel has traveled), limit switches (devices that stop a mechanism from
	moving any further when it hits the switch), double solenoids (pneumatics pistons), Differential
	Drive (prewritten code from WPI that makes driving simpler), etc.*/

	public static Spark leftDriveMotor;
	public static Spark rightDriveMotor;
	public static Spark liftMotor;
	public static Spark rotateIntakeMotor;
	public static Spark intakeMotor;
	public static DifferentialDrive hotWheels;
	public static Encoder leftEncoder;
	public static Encoder rightEncoder;
	public static DigitalInput upperLimitSwitch;
	public static DigitalInput lowerLimitSwitch;

	public static Compressor compressor;
	public static DoubleSolenoid pincherBitePiston;
	public static DoubleSolenoid pincherSlidePiston;
	public static DoubleSolenoid frontLiftPiston;
	public static DoubleSolenoid backLiftPiston;
	public static DoubleSolenoid cargoBoxPiston;
	
	public static SpeedControllerGroup thisIsJustATestIgnoreThis; //This is just a test, ignore this.
	
	public static void init() { /*Robot.java calls this method exactly once, so this is when all the
		variables above are turned into actual objects that the code can use. A common word to des-
		cribe the startup or activation of a device is "instantiation."*/

		//CHANGES NEEDED for everything
		leftDriveMotor = new Spark(0); /*Sparks are motor controllers. A Spark can receive a signal with
		a value of anything between 1 and -1, and that signal is relayed to the motor. 1 makes the
		motor spin at 100% speed in the clockwise direction. -1 makes the motor spin at 100% speed in
		the counterclockwise direction. 0 stops the motor. Anything in between allows the motor to
		spin at less than full speed.

		I need to go off on a few more tangents here because this is the first example of a construc-
		tor that we can see. A constructor is a method with the same name as a class, and when called
		it constructs an instance (example) of that class. In this case we are creating an instance
		of the Spark class that will be used in the future.

		The number in the parentheses is a parameter. A parameter offers additional information about
		the method when it is called. You can hover over a method to see the parameters that it
		requires. In this case it needs just one: a PWM port number. The number corresponds to the
		number of the PWM port on the RoboRio in which the Spark is plugged in. The number 0 means the
		Spark is plugged into the port labeled 0.*/

		//2017 and 2018 bot are 1
		leftDriveMotor.setInverted(true); /*Immediately after creating our Spark (which we will hence-
		forth refer to as "leftDriveMotor"), we call the method setInverted() (the method is inside
		the Spark class) and set it to true. This means the values are inverted--if the Spark receives
		a positive value, the motor will spin in the negative direction (counterclockwise), and vice
		versa. If we don't call this method, it defaults to false.*/
		
		rightDriveMotor = new Spark(1); // 2018 bot is 2, 2017 bot is 1
		rightDriveMotor.setInverted(true);
		
		hotWheels = new DifferentialDrive(leftDriveMotor, rightDriveMotor); /*The Differential Drive
		we see here uses two motors we've instantiated and links them together to form a Differential
		Drive. You can look into the specifics as to what a Differential Drive is by searching for it on
		FRC's website.*/
		
		liftMotor = new Spark(3);
		liftMotor.setInverted(true);
		
		rotateIntakeMotor = new Spark(4); 
		intakeMotor = new Spark(5); 
		intakeMotor.setInverted(true);
		
		/*Encoders measure the distance a wheel has traveled based on the number of revolutions it has
		made. The specifics of the encoders should be examined by searching on FRC's website. Also
		remember that you can mouse over the method to see what the parameters mean!*/
		leftEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
		leftEncoder.setDistancePerPulse(.1173); 
		leftEncoder.setMinRate(10);
		
		rightEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
		rightEncoder.setDistancePerPulse(.1173);
		rightEncoder.setMinRate(10);
		
		upperLimitSwitch = new DigitalInput(9);
		lowerLimitSwitch = new DigitalInput(8);

		compressor = new Compressor(0);
		compressor.setClosedLoopControl(true);
		
		pincherBitePiston = new DoubleSolenoid(0,0,1);
		pincherSlidePiston = new DoubleSolenoid(0,2,3);
		frontLiftPiston = new DoubleSolenoid(0,4,5);
		backLiftPiston = new DoubleSolenoid(0,6,7);
		cargoBoxPiston = new DoubleSolenoid(1,0,1);
		
	} /*After RobotMap.init() is finished running, the code goes back to Robot.java and continues on.
	Head back to Robot.java to see what happens next.*/
	
}
