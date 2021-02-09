// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import io.github.pseudoresonance.pixy2api.Pixy2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveBase_Subsystem;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Storage_Subsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here ...
  public static DriveBase_Subsystem driveBase_Subsystem;
  public static Intake_Subsystem intake_Subsystem;
  public static Storage_Subsystem storage_Subsystem;

  // After these, put object declarations (for cameras, ultrasonic, ...)

  public static Pixy2 cartridgePixy;          
  public static Pixy2 intakePixy;

  // Intake
  public static WPI_VictorSPX intakeVictor;
  public static Solenoid leftPneumaticPiston;
  public static Solenoid rightPneumaticPiston;

  // Storage
  public static WPI_VictorSPX lstorageVictor;
  public static WPI_VictorSPX rstorageVictor;

  // Drive Base
  public static WPI_TalonSRX m_leftDriveTalon;
  public static WPI_TalonSRX m_rightDriveTalon;
  public static WPI_VictorSPX m_frontLeftVic;
  public static WPI_VictorSPX m_backLeftVic;
  public static WPI_VictorSPX m_frontRightVic;
  public static WPI_VictorSPX m_backRightVic;

  public static SpeedControllerGroup m_leftDrive;
  public static SpeedControllerGroup m_rightDrive;

  public static DifferentialDrive CashwinsDriveBase;

  // create Joystick variable name
  public static Joystick XBoxController; // Static means that the method/class the variable or method belongs too
  // doesn't need to be created
  public static Joystick AssistantController;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    InitMap();

    intake_Subsystem = new Intake_Subsystem();
    storage_Subsystem = new Storage_Subsystem();
    driveBase_Subsystem = new DriveBase_Subsystem();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
  }

  public void InitMap() {
    // Drive Base Motor Initialization:
    m_leftDriveTalon = new WPI_TalonSRX(Constants.leftDriveTalonCAN); // other motor controllers will follow this
                                                                      // controller
    m_leftDriveTalon.set(ControlMode.Current, 0);
    m_leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10); // may need to
                                                                                                   // change configs
                                                                                                   // on MAG Encoder
    double leftDriveOut = m_leftDriveTalon.getSupplyCurrent();
    System.out.println("Left drive is going at :" + leftDriveOut);

    m_frontLeftVic = new WPI_VictorSPX(Constants.fLeftDriveVictorCAN);
    m_frontLeftVic.set(ControlMode.Follower, Constants.leftDriveTalonCAN);
    m_frontLeftVic.setInverted(true);

    m_backLeftVic = new WPI_VictorSPX(Constants.bLeftDriveVictorCAN);
    m_backLeftVic.set(ControlMode.Follower, Constants.leftDriveTalonCAN);

    m_rightDriveTalon = new WPI_TalonSRX(Constants.rightDriveTalonCAN); // other motor controllers will follow this
                                                                        // controller
    m_rightDriveTalon.set(ControlMode.Current, 0);
    m_rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10); // may need to
                                                                                                    // change
                                                                                                    // configs on
                                                                                                    // MAG Encoder

    double rightDriveOut = m_rightDriveTalon.getSupplyCurrent();
    System.out.println("Right drive is going at :" + rightDriveOut);     
        
    m_frontRightVic = new WPI_VictorSPX(Constants.fRightDriveVictorCAN);
    m_frontRightVic.set(ControlMode.Follower, Constants.rightDriveTalonCAN);

    m_backRightVic = new WPI_VictorSPX(Constants.bRightDriveVictorCAN);
    m_backRightVic.set(ControlMode.Follower, Constants.rightDriveTalonCAN);

    m_leftDrive = new SpeedControllerGroup(m_leftDriveTalon, m_frontLeftVic, m_backLeftVic); // Ports in order:
                                                                                             // 1, 2, 3
    m_rightDrive = new SpeedControllerGroup(m_rightDriveTalon, m_frontRightVic, m_backRightVic); // 4, 5, 6

    CashwinsDriveBase = new DifferentialDrive(m_leftDrive, m_rightDrive);

    // Init Intake Motors
    intakeVictor = new WPI_VictorSPX(Constants.intakeCAN);
    intakeVictor.set(ControlMode.Current, 0);

    // Init Storage Motors
    lstorageVictor = new WPI_VictorSPX(Constants.lstorageCAN);
    lstorageVictor.set(ControlMode.Current, 0);
    // may need to invert these motors

    rstorageVictor = new WPI_VictorSPX(Constants.rstorageCAN);
    rstorageVictor.set(ControlMode.Current, 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
        
  }
}
