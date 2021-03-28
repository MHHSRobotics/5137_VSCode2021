package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; //Import DifferentialDrive (a way to have an arcade drive)
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase; //Import Subsystem Class (*new this year*)
import frc.robot.Constants;
import frc.robot.Robot;
//New API may not need to import dependable commands
import frc.robot.RobotContainer; //Import Timed Robot methods (from overall robot)
import frc.robot.myPigeonIMU;
import frc.robot.commands.ArcadeDrive;

public class DriveBase_Subsystem extends SubsystemBase {
	//Differential Drive
	DifferentialDrive BMoneysDifferentialDrive;

	// Drive Motors
	SpeedControllerGroup m_leftDrive;
	SpeedControllerGroup m_rightDrive;
	TalonSRX PigeonTalon = new TalonSRX(7);

	TalonSRX leftDriveTalon = RobotContainer.m_leftDriveTalon;
	TalonSRX rightDriveTalon = RobotContainer.m_rightDriveTalon;

	Joystick XBoxController;

	double newDriveSpeed;
	double actualDriveSpeed;
	double previousDriveSpeed;

	// Gyro
	private final PigeonIMU m_gyro = new PigeonIMU(PigeonTalon);

	// Odometry
	private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(getRotation2d());

	public DriveBase_Subsystem() {
		BMoneysDifferentialDrive = RobotContainer.BMoneysDriveBase;

		m_leftDrive = RobotContainer.m_leftDrive;
		m_rightDrive = RobotContainer.m_rightDrive;

		XBoxController = RobotContainer.XBoxController;

		newDriveSpeed = 0;
		actualDriveSpeed = 0;
		previousDriveSpeed = 0;

		leftDriveTalon.setSelectedSensorPosition(Constants.EncoderDistancePerPulse);
		rightDriveTalon.setSelectedSensorPosition(Constants.EncoderDistancePerPulse);

		resetEncoders();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		//rampArcadeDrive(XBoxController);
		BMoneysDifferentialDrive.feed();
		m_odometry.update(
			getRotation2d(), -leftDriveTalon.getSelectedSensorPosition() 
							* Constants.EncoderDistancePerPulse, 
							rightDriveTalon.getSelectedSensorPosition() 
							* Constants.EncoderDistancePerPulse);
		//System.out.println("Left position: " + -leftDriveTalon.getSelectedSensorPosition() 
		//* Constants.EncoderDistancePerPulse);
		//System.out.println("Right Position: " + rightDriveTalon.getSelectedSensorPosition() 
		//* Constants.EncoderDistancePerPulse);
		
	}

	public void initDefaultCommand() {
		//setDefaultCommand(new ArcadeDrive());
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

		//BMoneysDifferentialDrive.curvatureDrive(-driveValue / Constants.driveSensitivity, turnValue / Constants.turnSensitivity, Constants.isQuickTurn);
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
			//BMoneysDifferentialDrive.curvatureDrive(0.0, Robot.targetx/Constants.turnSpeed, true);
            return false;
            
        }
        else if (Robot.targetx <= -Constants.marginAngleError) { //too far to the right
			//BMoneysDifferentialDrive.curvatureDrive(0.0, -Constants.turnRate, true); //turn clockwise
			//BMoneysDifferentialDrive.curvatureDrive(0.0, Robot.targetx/Constants.turnSpeed, true);
            return false;
        }
        else {

            //do this if the drive base is alligned 
            //BMoneysDifferentialDrive.curvatureDrive(-XBoxController.getRawAxis(Constants.LYStickAxisPort), 0.0, false);
            
			return true;
		}
            
        }

	public void drivePivot(double speed) { // TODO may need to make this negative
	//	BMoneysDifferentialDrive.arcadeDrive(0, speed);
	}

	public void driveStraight(double speed) {
	//	BMoneysDifferentialDrive.arcadeDrive(speed, 0);
	}

	public void stop() {
	//	BMoneysDifferentialDrive.arcadeDrive(0, 0);
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		// Selected sensor velocity return meters per 100 ms so multiply by 10/10
		return new DifferentialDriveWheelSpeeds(-leftDriveTalon.getSelectedSensorVelocity() * 10 * Constants.EncoderDistancePerPulse,   
												rightDriveTalon.getSelectedSensorVelocity() * 10 * Constants.EncoderDistancePerPulse); 
			
	}

	private void resetEncoders() {
		leftDriveTalon.setSelectedSensorPosition(0);
		rightDriveTalon.setSelectedSensorPosition(0);
	}

	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		m_odometry.resetPosition(pose, getRotation2d());
	}

	/*
	public void arcadeDrive(double fwd, double rot) {
		m_drive.arcadeDrive(fwd, rot);
	}
	*/

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		var batteryVoltage = RobotController.getBatteryVoltage();
    	if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
      		//leftVolts *= batteryVoltage / 12.0;
			  //rightVolts *= batteryVoltage / 12.0;
			leftVolts = leftVolts/12;
			rightVolts = rightVolts/12;
		}
		
		//leftVolts = Math.abs(leftVolts);
		//rightVolts = Math.abs(rightVolts);
		

		//leftVolts = leftVolts / Constants.scalingFactor;
		//rightVolts = rightVolts / Constants.scalingFactor;
		System.out.println("Tank drive volts: " + leftVolts + " : " + rightVolts 
							+ " Battery: " + batteryVoltage);

		m_leftDrive.setVoltage(leftVolts);
		m_rightDrive.setVoltage(-rightVolts);
	}

	public double getAverageEncoderDistance() {
		return (leftDriveTalon.getSelectedSensorPosition() + rightDriveTalon.getSelectedSensorPosition()) / 2.0;
	}

	public void zeroHeading() {
		resetGyro();
	}

	public double getHeading() {
		return getRotation2d().getDegrees();
	}

	public Rotation2d getRotation2d() {
        double [] ypr = new double[3];

		m_gyro.getYawPitchRoll(ypr);
		//System.out.println("Yaw Values: " + ypr[0]);
		return Rotation2d.fromDegrees(ypr[0]);
	}
	
	public void resetGyro() {
        m_gyro.setYaw(0);
    }

	
	public double getTurnRate() {
		double [] ypr = new double[3];

		m_gyro.getRawGyro(ypr);
		return ypr[0];
	}
	
	
}