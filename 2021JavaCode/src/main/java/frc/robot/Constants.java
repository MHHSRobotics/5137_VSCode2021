// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Used for OI
    public final static int JoystickPort = 0;
    public final static int AssistantJoystickPort = 1;
    //---------------------------------------//
    public final static int AButtonPort = 1;
    public final static int BButtonPort = 2;
    public final static int XButtonPort = 3;
    public final static int YButtonPort = 4;
    public final static int LBButtonPort = 5;
    public final static int RButtonPort = 6;
    public final static int StartButtonPort = 7;
    public final static int SelectButtonPort = 8;
    //---------------------------------------//
    public final static int LXStickAxisPort = 0;
    public final static int LYStickAxisPort = 1;
    public final static int LTAxisPort = 2;
    public final static int RTAxisPort = 3;
    public final static int RXStickAxisPort = 4;
    public final static int RYStickAxisPort = 5;

    //---------------------------------------//

    //Can Items                                                                                           
    //Drive Base                                        //Labels: A=right & B=left                         
    public final static int leftDriveTalonCAN = 1;      //B2                                              
    public final static int rightDriveTalonCAN = 2;     //A2                                              
    public final static int fLeftDriveVictorCAN = 3;    //B3                                              
    public final static int fRightDriveVictorCAN = 4;   //A3                                              
    public final static int bLeftDriveVictorCAN = 5;    //B4                                              
    public final static int bRightDriveVictorCAN = 6;   //A4                                              

    //Intake                                                                                               
    public final static int intakeCAN = 8;              //C8                                                    
    public static final int PCMCAN = 41;                //No label.PCM CAN items are subject to change     
    public static final int PDPCAN = 40;                //No Label                                         
    public static final int leftIntakePiston = 0; 
    public static final int rightIntakePiston = 1;

    //Storage
    public final static int lstorageCAN = 11;           //C4
    public final static int rstorageCAN = 12;           //C3

    //---------------------------------------//

    //Shooter
    public final static double marginAngleError = 1.25; //amount in degrees that robot is allowed to be offset from target when shooting
    public final static double turnRate = 0.2; //turn speed based on percentage of drive base output
    public final static double turnSpeed = 40.0; //turn speed based on porportion from targetx (doesn't oscolate) (maximum angle is 28 degrees)

    //Drive Base
    public final static double driveSensitivity = 1.0; //bigger # means less sensitivity, from 0.5 to 2.0
    //10.0: baby speed, 9.0: toddler mode, 7.0: fast toddler mode, 5.0: optimal turn speed, 4.5:
    public final static double turnSensitivity = 3.0; //4.5 seems nice
    public final static boolean isQuickTurn = true; //makes turning the drive base able to override constant-curvature turning for turn-in-place maneuvers.

    //Intake 
    public final static double startingBallCount = 3.0;
    public final static double intakeVictorOutput = 0.3;
    public final static int intakeWaitTime = 10;        //wait time in ms to see if the storage is full

    //Storage
    public final static double storageSpeed = 0.85; //speed of the belts in PERCENT (future need to change to amps)!!!
    public final static int pixyLEDBrightness = 255;
    public final static int storageWaitTime = 2000; //wait time to allow belt motors to continue for until it finds the target (in ms)

    // Encoders
	public static final int[] kLeftEncoderPorts = {7, 8}; // Will need to be changed ARE WRONG Find correct ports
	public static final boolean kLeftEncoderReversed = true;
	public static final int[] kRightEncoderPorts = {9, 10}; // Will need to be changed ARE WRONG Find correct ports
    public static final boolean kRightEncoderReversed = false;

    // Ramsete Command
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kTrackwidthMeters = 0.625;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 5;

    public static final double ksVolts = 0.22;     
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2; // ABSOLUTLELY CRITICAL THAT THESE VALUES BE CHANGED
    public static final double kPDriveVel = 8.5;
}
