package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; //Import DifferentialDrive (a way to have an arcade drive)
import edu.wpi.first.wpilibj2.command.SubsystemBase; //Import Subsystem Class (*new this year*)
import frc.robot.Constants;
import frc.robot.Robot;
//New API may not need to import dependable commands
import frc.robot.RobotContainer; //Import Timed Robot methods (from overall robot)
import frc.robot.commands.ArcadeDrive;

public class DriveBase_Subsystem extends SubsystemBase {
	DifferentialDrive BMoneysDifferentialDrive;

	SpeedControllerGroup m_leftDrive;
	SpeedControllerGroup m_rightDrive;

	Joystick XBoxController;

	double newDriveSpeed;
	double actualDriveSpeed;
	double previousDriveSpeed;

	public DriveBase_Subsystem() {
		BMoneysDifferentialDrive = RobotContainer.BMoneysDriveBase;

		m_leftDrive = RobotContainer.m_leftDrive;
		m_rightDrive = RobotContainer.m_rightDrive;

		XBoxController = RobotContainer.XBoxController;

		newDriveSpeed = 0;
		actualDriveSpeed = 0;
		previousDriveSpeed = 0;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		rampArcadeDrive(XBoxController);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new ArcadeDrive());
	}

	public double adjustJoystickValue(double joystick, double deadZone) {
		double adjustedJoystick;
		if (Math.abs(joystick) < deadZone) {
			adjustedJoystick = 0;
		} else {
			adjustedJoystick = ((1 / (1 - deadZone)) * (joystick - deadZone));
		}
		return adjustedJoystick;
		// An algorithm developed by the fantastic Sarah C. Lincoln that adjusts the
		// joysticks
		// to run scaled to the deadZone
	}

	public void rampArcadeDrive(Joystick XBoxController) {
		double driveValue = XBoxController.getRawAxis(Constants.LYStickAxisPort);
		//System.out.println("LYStick Value: " + driveValue);
		double turnValue = XBoxController.getRawAxis(Constants.RXStickAxisPort);
		//System.out.println("RXStick Value: " + turnValue);

		/*
			newDriveSpeed = accelerate(driveValue, Constants.minSpeed,
			Constants.maxSpeed, previousDriveSpeed, Constants.accelFactor);
			actualDriveSpeed = newDriveSpeed;
			previousDriveSpeed = actualDriveSpeed; 
		*/

		//want to add if statement here to have configs
		//BMoneysDifferentialDrive.arcadeDrive(-driveValue / Constants.driveSensitivity, turnValue / Constants.turnSensitivity); //non-car drive

		BMoneysDifferentialDrive.curvatureDrive(-driveValue / Constants.driveSensitivity, turnValue / Constants.turnSensitivity, Constants.isQuickTurn);
		//System.out.println("LeftTalon running at: A" + RobotContainer.m_leftDriveTalon.getSupplyCurrent());
		//System.out.println("RightTalon running at: A" + RobotContainer.m_rightDriveTalon.getSupplyCurrent()); //IDK about victors
	}

	/*public double accelerate(double driveValue, double minSpeed, double maxSpeed, double previousSpeed, double accelFactor) { // NEEDS TO BE TESTED
		double newSpeed;
		 
		
		return newSpeed;
	} */

	public boolean orientHorizontalTurn() { //returns true if the robot is horizontally oriented, false if interrupted
        if (Robot.targetx >= Constants.marginAngleError) { //if too far to the left
			//BMoneysDifferentialDrive.curvatureDrive(0.0, Constants.turnRate, true); //turn cntrclockwise
			BMoneysDifferentialDrive.curvatureDrive(0.0, Robot.targetx/Constants.turnSpeed, true);
            return false;
            
        }
        else if (Robot.targetx <= -Constants.marginAngleError) { //too far to the right
			//BMoneysDifferentialDrive.curvatureDrive(0.0, -Constants.turnRate, true); //turn clockwise
			BMoneysDifferentialDrive.curvatureDrive(0.0, Robot.targetx/Constants.turnSpeed, true);
            return false;
        }
        else {

            //do this if the drive base is alligned 
            BMoneysDifferentialDrive.curvatureDrive(-XBoxController.getRawAxis(Constants.LYStickAxisPort), 0.0, false);
            
			return true;
		}
            
        }

	public void drivePivot(double speed) { // TODO may need to make this negative
		BMoneysDifferentialDrive.arcadeDrive(0, speed);
	}

	public void driveStraight(double speed) {
		BMoneysDifferentialDrive.arcadeDrive(speed, 0);
	}

	public void stop() {
		BMoneysDifferentialDrive.arcadeDrive(0, 0);
	}
}