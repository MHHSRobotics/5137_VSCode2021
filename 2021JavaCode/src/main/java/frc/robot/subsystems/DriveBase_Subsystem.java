// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; //Import DifferentialDrive (a way to have an arcade drive)
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase; //Import Subsystem Class (*new this year*)
import frc.robot.Constants;
import frc.robot.Robot;
//New API may not need to import dependable commands
import frc.robot.RobotContainer; //Import Timed Robot methods (from overall robot)
import frc.robot.commands.ArcadeDrive;

public class DriveBase_Subsystem extends SubsystemBase {
	DifferentialDrive CashwinsDifferentialDrive;

	SpeedControllerGroup m_leftDrive;
	SpeedControllerGroup m_rightDrive;

	Joystick XBoxController;

	// The left-side drive encoder
	private final Encoder m_leftEncoder =
	new Encoder(Constants.kLeftEncoderPorts[0], Constants.kLeftEncoderPorts[1],
				Constants.kLeftEncoderReversed);

	// The right-side drive encoder
	private final Encoder m_rightEncoder =
	new Encoder(Constants.kRightEncoderPorts[0], Constants.kRightEncoderPorts[1],
				Constants.kRightEncoderReversed);

	DifferentialDriveOdometry m_odometry;
	Gyro m_gyro;

	double newDriveSpeed;
	double actualDriveSpeed;
	double previousDriveSpeed;

	public DriveBase_Subsystem() {
		CashwinsDifferentialDrive = RobotContainer.CashwinsDriveBase;

		m_leftDrive = RobotContainer.m_leftDrive;
		m_rightDrive = RobotContainer.m_rightDrive;

		XBoxController = RobotContainer.XBoxController;

		m_gyro = RobotContainer.gyro;
		m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

		newDriveSpeed = 0;
		actualDriveSpeed = 0;
		previousDriveSpeed = 0;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		rampArcadeDrive(XBoxController);
		m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(),
                      m_rightEncoder.getDistance());
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
		//CashwinsDifferentialDrive.arcadeDrive(-driveValue / Constants.driveSensitivity, turnValue / Constants.turnSensitivity); //non-car drive

		CashwinsDifferentialDrive.curvatureDrive(-driveValue / Constants.driveSensitivity, turnValue / Constants.turnSensitivity, Constants.isQuickTurn);
		//System.out.println("LeftTalon running at: A" + RobotContainer.m_leftDriveTalon.getSupplyCurrent());
		//System.out.println("RightTalon running at: A" + RobotContainer.m_rightDriveTalon.getSupplyCurrent()); //IDK about victors
	}

	/*public double accelerate(double driveValue, double minSpeed, double maxSpeed, double previousSpeed, double accelFactor) { // NEEDS TO BE TESTED
		double newSpeed;
		 
		
		return newSpeed;
	} */

  /*
	public boolean orientHorizontalTurn() { //returns true if the robot is horizontally oriented, false if interrupted
        if (Robot.targetx >= Constants.marginAngleError) { //if too far to the left
			//CashwinsDifferentialDrive.curvatureDrive(0.0, Constants.turnRate, true); //turn cntrclockwise
			CashwinsDifferentialDrive.curvatureDrive(0.0, Robot.targetx/Constants.turnSpeed, true);
            return false;
            
        }
        else if (Robot.targetx <= -Constants.marginAngleError) { //too far to the right
			//CashwinsDifferentialDrive.curvatureDrive(0.0, -Constants.turnRate, true); //turn clockwise
			CashwinsDifferentialDrive.curvatureDrive(0.0, Robot.targetx/Constants.turnSpeed, true);
            return false;
        }
        else {

            //do this if the drive base is alligned 
            CashwinsDifferentialDrive.curvatureDrive(-XBoxController.getRawAxis(Constants.LYStickAxisPort), 0.0, false);
            return true;
		    }
            
  }
  */
  
	public void drivePivot(double speed) { // TODO may need to make this negative
		CashwinsDifferentialDrive.arcadeDrive(0, speed);
	}

	public void driveStraight(double speed) {
		CashwinsDifferentialDrive.arcadeDrive(speed, 0);
	}

	public void stop() {
		CashwinsDifferentialDrive.arcadeDrive(0, 0);
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		m_leftDrive.setVoltage(leftVolts);
		m_rightDrive.setVoltage(-rightVolts);
		CashwinsDifferentialDrive.feed();
	}

	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		m_odometry.resetPosition(pose, m_gyro.getRotation2d());
	}

	public void resetEncoders() {
		m_leftEncoder.reset();
		m_rightEncoder.reset();
	}
}
