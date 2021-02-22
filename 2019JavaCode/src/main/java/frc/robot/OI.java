package frc.robot;

import frc.robot.commands.CargoBox_Command;
//import frc.robot.commands.GetDistance;
import frc.robot.commands.LiftRobot_Command;
import frc.robot.commands.LowerBack_Command;
import frc.robot.commands.LowerFront_Command;
import frc.robot.commands.PincherBite_Command;
import frc.robot.commands.PincherSlide_Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI { /*Welcome to the OI. This class is set aside to create our buttons and assign them
	to our controllers. We start by declaring our controller and button variables, as always.*/
	
	public Joystick jackBlack;
	public Joystick redDead;

	public JoystickButton getDistanceButton;
	public JoystickButton bitePincherButton;
	public JoystickButton slidePincherButton;
	public JoystickButton liftRobotButton;
	public JoystickButton lowerFrontButton;
	public JoystickButton lowerBackButton;
	public JoystickButton cargoBoxButton;
	public JoystickButton theRobotIsBeingSoUnbelievablyStupidThatWeNeedToShutDownLiterallyEverythingButton;
	
	public OI() {
		/*The code here actually creates two controllers, named "jackBlack" and "redDead". The port num-
		ber corresponds to the order in which the controllers appear in the FRC Driver Station. Open
		that up now and click the tab on the left that has a USB symbol. Now plug in a controller into
		a USB port on your computer. You should see the controller appear in the box labeled "USB
		Order" at port number 0. That is the port which the Joystick() parameter refers to. This tab
		also allows you to see what numbers the axes and buttons are labeled. Move an analog stick and
		you should see an axis change value. Press a button and you should see a box light up. The
		number of a button is determined by starting at the top left box, which is number 1, and then
		counting up by 1 for each box you move downward. (Example: The A button lights up the top left
		box. This means A is button 1. The B button causes the second box to light up, which means B
		is button 2.)*/
		jackBlack = new Joystick(0);
		redDead = new Joystick(1);

		/*I'll explain everything up here to keep the clutter out of there. We start by creating a new
		JoystickButton() with the two parameters being the joystick it belongs to and the number of the
		button. The next line allows us to tell the robot what the button does. Let's take the
		bitePincherButton as an example:

		bitePincherButton.whenPressed(new PincherBite_Command());

		We access the method "whenPressed()" within the JoystickButton class. This is WPI code which
		will execute the method whenever the button is pressed. In this case it constructs and then
		runs PincherBite_Command. The specifics as to how many times it runs and when it stops isn't
		made too specific when using whenPressed(), so you'll see what's going on when you visit the
		command later.

		whenPressed() is not the only way you can use the button. There are many other possibilities
		available, such as whileHeld() (which only makes the command run while the button is held, and
		deactivates when the button is released), toggleWhenPressed() (which keeps the command active
		after pressing the button once, then deactivates after pressing a second time, then on again,
		etc.), and many others. You can see all the methods that you can choose from by deleting
		".whenPressed" and then re-typing the period.
		
		Return to Robot.java to see what happens after the OI is constructed.*/
	

		//The code below is for one XBox controller. Add a /* at the beginning
		//and */ at the end to deactivate this code, and remove them to activate.
		
		//START

		/*

		bitePincherButton = new JoystickButton(jackBlack, 5); //Left Bumper
		bitePincherButton.whenPressed(new PincherBite_Command());
		
		slidePincherButton = new JoystickButton(jackBlack, 6); //Right Bumper
		slidePincherButton.whenPressed(new PincherSlide_Command());
		
		liftRobotButton = new JoystickButton(jackBlack, 4); //Y
		liftRobotButton.whileHeld(new LiftRobot_Command());
		
		lowerFrontButton = new JoystickButton(jackBlack, 3); //X
		lowerFrontButton.whileHeld(new LowerFront_Command());
		
		lowerBackButton = new JoystickButton(jackBlack, 2); //B
		lowerBackButton.whileHeld(new LowerBack_Command());

		cargoBoxButton = new JoystickButton(jackBlack, 1); //A
		cargoBoxButton.whenPressed(new CargoBox_Command());

		//Other Controls:
		//Drive Straight = Left Analog Up/Down
		//Turn = Right Analog Left/Right
		//Drive Lift = Right Analog Up/Down
		
		*/

		//END

		//The code below is for two XBox controllers. Add a /* at the beginning
		//and */ at the end to deactivate this code, and remove them to activate.
		//Drive Base and Lift are port 0; everything else is port 1.

		//START

		

		liftRobotButton = new JoystickButton(jackBlack, 4); //Y, 0
		liftRobotButton.whileHeld(new LiftRobot_Command());
		
		lowerFrontButton = new JoystickButton(jackBlack, 3); //X, 0
		lowerFrontButton.whileHeld(new LowerFront_Command());
		
		lowerBackButton = new JoystickButton(jackBlack, 2); //B, 0
		lowerBackButton.whileHeld(new LowerBack_Command());

		bitePincherButton = new JoystickButton(redDead, 5); //Left Bumper, 1
		bitePincherButton.whenPressed(new PincherBite_Command());
		
		slidePincherButton = new JoystickButton(redDead, 6); //Right Bumper, 1
		slidePincherButton.whenPressed(new PincherSlide_Command());

		cargoBoxButton = new JoystickButton(redDead, 1); //A, 1
		cargoBoxButton.whenPressed(new CargoBox_Command());

		//Other Controls:
		//Drive Straight = Left Analog Up/Down, 0
		//Turn = Right Analog Left/Right, 0
		//Drive Lift = Right Analog Up/Down, 0



		//END

	}
	
}
