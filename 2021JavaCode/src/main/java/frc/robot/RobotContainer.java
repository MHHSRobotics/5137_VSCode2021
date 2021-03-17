package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MyRamseteCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoShoot_Command;
import frc.robot.commands.ClimbDown_Command;
import frc.robot.commands.ClimbUp_Command;
import frc.robot.commands.DownwardStorage_Command;
import frc.robot.commands.ManShoot_Command;
import frc.robot.commands.OffIntake_Command;
import frc.robot.commands.OffStorage_Command;
import frc.robot.commands.OnIntake_Command;
import frc.robot.commands.ReversedOnIntake_Command;
import frc.robot.commands.StahpTheShoot_Command;
import frc.robot.commands.StopClimb_Command;
import frc.robot.commands.StopShootStorage_Command;
import frc.robot.commands.UpwardStorage_Command;
import frc.robot.subsystems.Climb_Subsystem;
import frc.robot.subsystems.DriveBase_Subsystem;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Shooter_Subsystem;
import frc.robot.subsystems.Storage_Subsystem;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.SPILink;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 * 
 * If you aren't familiar with this (since this is new to this year), this is a
 * combination of OI.java and RobotMap.java, both of which are gone this this
 * year into this class. If desired, this class can be subdivided back into the
 * previous two files.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // Commands go here...

    // Subsystems (actual object name, not class names)
    public static Climb_Subsystem climb_Subsystem;
    public static DriveBase_Subsystem driveBase_Subsystem;
    public static Intake_Subsystem intake_Subsystem;
    public static Shooter_Subsystem shooter_Subsystem;
    public static Storage_Subsystem storage_Subsystem;

    // After these, put object declarations (for cameras, ultrasonic, ...)

    public static Pixy2 cartridgePixy;
    public static Pixy2 intakePixy;
    
    // Motor Controllers:

    // Drive Base
    public static WPI_TalonSRX m_leftDriveTalon;
    public static WPI_TalonSRX m_rightDriveTalon;
    public static WPI_VictorSPX m_frontLeftVic;
    public static WPI_VictorSPX m_backLeftVic;
    public static WPI_VictorSPX m_frontRightVic;
    public static WPI_VictorSPX m_backRightVic;

    public static SpeedControllerGroup m_leftDrive;
    public static SpeedControllerGroup m_rightDrive;

    public static DifferentialDrive BMoneysDriveBase;

    // Shooter
    public static WPI_TalonSRX shooterTalon;
    public static WPI_TalonSRX followerShooterTalon;

    // Control Panel
    public static TalonSRX controlPanelTalon;

    // Intake
    public static WPI_VictorSPX intakeVictor;

    // Storage
    public static WPI_VictorSPX lstorageVictor;
    public static WPI_VictorSPX rstorageVictor;

    // Climb
    public static WPI_TalonSRX climbTalon;

    // Solenoids:
    public static Compressor compressor;

    // Intake
    public static Solenoid leftPneumaticPiston;
    public static Solenoid rightPneumaticPiston;

    // Shooter
    public static Solenoid shootPneumaticPistonOne;
    public static Solenoid shootPneumaticPistonTwo;
    public static Solenoid shootPneumaticPistonThree;

    // Climb

    // public static DoubleSolenoid climbPiston;

    // Switches

    // Limit Switches
    // public static DigitalInput limitSwitch;

    // Sensors
    public static I2C.Port i2cPort = I2C.Port.kOnboard;
    public static SPILink spiLink;

    // Joystick buttons
    public static JoystickButton AButton; // A
    public static JoystickButton BButton; // B
    public static JoystickButton XButton; // ...
    public static JoystickButton YButton;
    public static JoystickButton SelectButton;
    public static JoystickButton StartButton;

    public static JoystickButton AsLBButton; // Left bumper button
    public static JoystickButton AsRBButton; // ...
    public static JoystickButton XbLBButton;
    public static JoystickButton XbRBButton;

    // Triggers
    public static Trigger XrTrigger;
    public static Trigger XlTrigger;
    public static Trigger ArTrigger;
    public static Trigger AlTrigger;

    // DPad Buttons (names)
    public static POVButton uDPadButton; // Up DPad
    public static POVButton dDPadButton; // Down DPad
    public static POVButton lDPadButton; // Left DPad
    public static POVButton rDPadButton; // Right DPad
    public static POVButton ulDPadButton; // Up-Left DPad
    public static POVButton urDPadButton; // Up-Right DPad
    public static POVButton dlDPadButton; // Down-Left DPad
    public static POVButton drDPadButton; // Down-Right DPad
    public static POVButton nDPadButton; // no press on DPad

    // create Joystick variable name
    public static Joystick XBoxController; // Static means that the method/class the variable or method belongs too
                                           // doesn't need to be created
    public static Joystick AssistantController;

    // create SmartDashboard name
    public static SmartDashboard smartDashboard;
    public static ShuffleboardTab diagnosticTab;
    public static ShuffleboardTab liveWindowTab;

    public static Timer timer;

    public static DigitalInput LimitSwitchUpper;
    public static DigitalInput LimitSwitchLower;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        // Similar to RobotMap, now we are going to create all the subsystem objects
        // (for commands)
        InitMap();
        /*controlPanel_Subsystem = new ControlPanel_Subsystem();

        */
        //Needs to 6be here (don't ask, im mad)
 
        XBoxController = new Joystick(Constants.JoystickPort);

        AssistantController = new Joystick(Constants.AssistantJoystickPort);

    
        //climb_Subsystem = new Climb_Subsystem();
        intake_Subsystem = new Intake_Subsystem();
        storage_Subsystem = new Storage_Subsystem();
        shooter_Subsystem = new Shooter_Subsystem();
        driveBase_Subsystem = new DriveBase_Subsystem();
        climb_Subsystem = new Climb_Subsystem();
        // Configure the button bindings
        configureButtonBindings();
        
        // Set creation of other objects (like cameras)

        // For cameras, set defaults here (like resolution, framerate, ...)

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     * 
     * This is the exact same as OI from previous years.
     */
    private void configureButtonBindings() {

        // Test this, may be good for using axes
        
        BooleanSupplier booleanSupplyXBoxRT = () -> {
            if (XBoxController.getRawAxis(Constants.RTAxisPort) > 0.1 && XBoxController.getRawAxis(Constants.LTAxisPort) < 0.1)
            {
                return true;
            } else {
                return false;
            }
        };
        BooleanSupplier booleanSupplyXBoxLT = () -> {
            if (XBoxController.getRawAxis(Constants.LTAxisPort) > 0.1 && XBoxController.getRawAxis(Constants.RTAxisPort) < 0.1)
            {
                return true;
            } else {
                return false;
            }
        };
        BooleanSupplier booleanSupplyAssistantRT = () -> {
            if (AssistantController.getRawAxis(Constants.RTAxisPort) > 0.1 && XBoxController.getRawAxis(Constants.LTAxisPort) < 0.1) {
                return true;
            } else {
                return false;
            }
        }; 
        BooleanSupplier booleanSupplyAssistantLT = () -> {
            if (AssistantController.getRawAxis(Constants.LTAxisPort) > 0.1 && XBoxController.getRawAxis(Constants.RTAxisPort) < 0.1) {
                return true;
            } else {
                return false;
            }
        }; 

        //Automagic Shooter Control
        XrTrigger = new Trigger(booleanSupplyXBoxRT);
        XrTrigger.whileActiveContinuous(new AutoShoot_Command());//makes automatic shooter engage
        // NOTE: this version of whileActiveContinuous override
        // any and all drive base control. May need to change.
        XrTrigger.whileActiveOnce(new ArcadeDrive()); //returns regular drivebase control (will toggle between regular and shooter driving)
        XrTrigger.whenInactive(new StahpTheShoot_Command());
        XrTrigger.whenInactive(new StopShootStorage_Command());

        //Manual Shooter Control
        XlTrigger = new Trigger(booleanSupplyXBoxLT);
        XlTrigger.whileActiveContinuous(new ManShoot_Command()); //makes manual shooter engage
        XlTrigger.whileActiveOnce(new ArcadeDrive()); //may need to change
        XlTrigger.whenInactive(new StahpTheShoot_Command());
        XlTrigger.whenInactive(new StopShootStorage_Command());

        //Manual Shooter Control (upDpad = initiation line) (rDpad = front trench) (dDpad = back trench)

        //Up DPAD
        uDPadButton = new POVButton(XBoxController, Constants.uDPadButtonValue); //uDPadButtonValue
        uDPadButton.whileActiveContinuous(new ManShoot_Command()); //makes manual shooter engage
        uDPadButton.whileActiveOnce(new ArcadeDrive()); //may need to change
        uDPadButton.whenInactive(new StahpTheShoot_Command());
        uDPadButton.whenInactive(new StopShootStorage_Command());

        rDPadButton = new POVButton(XBoxController, Constants.rDPadButtonValue);
        rDPadButton.whileActiveContinuous(new ManShoot_Command()); //makes manual shooter engage
        rDPadButton.whileActiveOnce(new ArcadeDrive()); //may need to change
        rDPadButton.whenInactive(new StahpTheShoot_Command());
        rDPadButton.whenInactive(new StopShootStorage_Command());

        dDPadButton = new POVButton(XBoxController, Constants.dDPadButtonValue);
        dDPadButton.whileActiveContinuous(new ManShoot_Command()); //makes manual shooter engage
        dDPadButton.whileActiveOnce(new ArcadeDrive()); //may need to change
        dDPadButton.whenInactive(new StahpTheShoot_Command());
        dDPadButton.whenInactive(new StopShootStorage_Command());

        AlTrigger = new Trigger(booleanSupplyAssistantLT);
        AlTrigger.whileActiveContinuous(new OnIntake_Command());
        AlTrigger.whileActiveContinuous(new UpwardStorage_Command());
        AlTrigger.whenInactive(new OffIntake_Command());
        AlTrigger.whenInactive(new OffStorage_Command());

        ArTrigger = new Trigger(booleanSupplyAssistantRT);
        ArTrigger.whileActiveContinuous(new ReversedOnIntake_Command());
        ArTrigger.whileActiveContinuous(new DownwardStorage_Command());
        ArTrigger.whenInactive(new OffStorage_Command());
        ArTrigger.whenInactive(new OffIntake_Command());

        AsRBButton = new JoystickButton(AssistantController, Constants.RButtonPort);
        AsRBButton.whileActiveContinuous(new ReversedOnIntake_Command());
        AsRBButton.whenInactive(new OffIntake_Command());
        AsRBButton.whenInactive(new OffStorage_Command());

        AsLBButton = new JoystickButton(AssistantController, Constants.LBButtonPort);
        AsLBButton.whileActiveContinuous(new UpwardStorage_Command());
        AsLBButton.whenInactive(new OffIntake_Command());
        AsLBButton.whenInactive(new OffStorage_Command());
        
        XbLBButton = new JoystickButton(XBoxController, Constants.LBButtonPort);
        XbLBButton.whileActiveContinuous(new ClimbUp_Command());
        XbLBButton.whenInactive(new StopClimb_Command());

        XbRBButton = new JoystickButton(XBoxController, Constants.RButtonPort);
        XbRBButton.whileActiveContinuous(new ClimbDown_Command());
        XbRBButton.whenInactive(new StopClimb_Command());

        // Sets B Button to do Control Panel Command
        //BButton = new JoystickButton(XBoxController, Constants.BButtonPort);
        //BButton.whenHeld(new ControlPanel_Command());
        
    }

    public void InitMap() {

        timer = new Timer();

        // Shuffle Board initialization:
        diagnosticTab = Shuffleboard.getTab("Diagnostics");
        liveWindowTab = Shuffleboard.getTab("Live Window");
        SmartDashboard.putNumber("Ball Count", Constants.startingBallCount);

        // Drive Base Moter Initialization:
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

        BMoneysDriveBase = new DifferentialDrive(m_leftDrive, m_rightDrive);
        System.out.println("Created differential drive");

        // Init ControlPanel Motors
        controlPanelTalon = new TalonSRX(Constants.controlPanelCAN);
        //controlPanelTalon.set(ControlMode.Velocity, 0);
        //controlPanelTalon.setInverted(true);
        //controlPanelTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute); // may need to
                                                                                                        // change
                                                                                                        // configs on
                                                                                                        // MAG Encoder

        // Init Intake Motors
        intakeVictor = new WPI_VictorSPX(Constants.intakeCAN);
        intakeVictor.set(ControlMode.Current, 0);

        // Init Shooter Motors
        shooterTalon = new WPI_TalonSRX(Constants.shooterCAN);
        shooterTalon.setInverted(false);
        shooterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); // may need to change
                                                                                                   // configs on MAG
        shooterTalon.selectProfileSlot(Constants.kPIDLoopIdx, 0);
        shooterTalon.config_kF(Constants.kPIDLoopIdx, 0.0330, Constants.kTimeoutMs);
        shooterTalon.config_kP(Constants.kPIDLoopIdx, 0.2, Constants.kTimeoutMs);
        shooterTalon.config_kI(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);
        shooterTalon.config_kD(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);
        shooterTalon.config_IntegralZone(Constants.kPIDLoopIdx, 100, 0);
        shooterTalon.configClosedLoopPeakOutput(Constants.kPIDLoopIdx, 1.0, Constants.kTimeoutMs);
        shooterTalon.setSensorPhase(true); // Encoder is "flipped"
        shooterTalon.set(ControlMode.Velocity, 0);

        followerShooterTalon = new WPI_TalonSRX(Constants.followerShooterCAN);
        followerShooterTalon.configClosedLoopPeakOutput(Constants.kPIDLoopIdx, 1.0, Constants.kTimeoutMs);
        followerShooterTalon.selectProfileSlot(Constants.kPIDLoopIdx, 0);
        followerShooterTalon.set(ControlMode.Follower, Constants.shooterCAN);

        // Init Storage Motors
        lstorageVictor = new WPI_VictorSPX(Constants.lstorageCAN);
        lstorageVictor.set(ControlMode.Current, 0);
        // may need to invert these motors

        rstorageVictor = new WPI_VictorSPX(Constants.rstorageCAN);
        rstorageVictor.set(ControlMode.Current, 0);

        // Init Climb Motors (needs to change)
        climbTalon = new WPI_TalonSRX(Constants.lclimbCAN);
        climbTalon.set(ControlMode.Current, 0);
        climbTalon.setInverted(true);

        // Sensor Init

        LimitSwitchUpper = new DigitalInput(Constants.LimitSwitchUpperDIOPort);
        LimitSwitchLower = new DigitalInput(Constants.LimitSwitchLowerDIOPort);

        /*
        spiLink = new SPILink();
        spiLink.open()
        cartridgePixy = Pixy2.createInstance(new SPILink()); // Creates a new Pixy2 camera using SPILink (need to add another one for Uart)
        
        //Pixy Setup code
        cartridgePixy.setCameraBrightness(Constants.pixyLEDBrightness);
        /*
        intakePixy = Pixy2.createInstance(new Link()); //NOTE: LINKS ARE SUBJECT TO CHANGE!!!!!!
        intakePixy.setCameraBrightness(Constants.pixyLEDBrightness); 
        */ 
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    
    public static Command getAutonomousCommand(String m_autoSelected) {
    // An ExampleCommand will run in autonomous
    /*

    ShootDriveBack_Command shootDriveBack = new ShootDriveBack_Command();
    DriveBack_Command driveBack_Command = new DriveBack_Command();
    ManShootDriveBack_Command manShootDriveBack_Command = new ManShootDriveBack_Command();

    //This might go here...
    
    switch (m_autoSelected) {
        case (Constants.ShootDriveBack): //command to shoot 3 balls, then drive back for a half-second
          return shootDriveBack;
        case (Constants.JustDriveBack):
          return driveBack_Command;
        case (Constants.ManShootDriveBack):
          return manShootDriveBack_Command;
        default:
          return manShootDriveBack_Command;

    //Does the data type need to be a command? Or is this good?

    } 
    */

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
    
    // An example trajectory to follow.  All units in meters.
    String trajectoryJSON = "Paths/Slalom.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    //trajectory.addConstraint(autoVoltageConstraint);

    //BMoneysDriveBase.setSafetyEnabled(false);
    driveBase_Subsystem.zeroHeading();

    MyRamseteCommand ramseteCommand =
        new MyRamseteCommand(
            trajectory,
            driveBase_Subsystem::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            driveBase_Subsystem::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            driveBase_Subsystem::tankDriveVolts,
            driveBase_Subsystem);

    // Reset odometry to the starting pose of the trajectory.
    driveBase_Subsystem.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveBase_Subsystem.tankDriveVolts(0, 0));

    }
}
