package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter_Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousManShoot_Command extends CommandBase {

    boolean dontSpam = true;
    int BallsShotCounter = 0;
    int shootXAmntBalls;

    Timer m_timer;
    double ourTime;

    public AutonomousManShoot_Command(int shootXBalls, double time) {
      m_timer = new Timer();
      ourTime = time;
        shootXAmntBalls = shootXBalls;
        addRequirements(RobotContainer.shooter_Subsystem);
        addRequirements(RobotContainer.driveBase_Subsystem);
        addRequirements(RobotContainer.storage_Subsystem); //necessary?
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      System.out.println("Shooter Be shootin");
      m_timer.reset();
      m_timer.start();
      //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      //table.getEntry("pipeline").setNumber(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.shooter_Subsystem.shoot(Constants.shooterAngle, false, true, true) == true) {//ready to shoot {
      RobotContainer.storage_Subsystem.store(true, true, false, false, false);
      if (dontSpam) {
        BallsShotCounter++;
        dontSpam = false;
    }
    }
    else {
      RobotContainer.storage_Subsystem.store(true, false, false, false, false);
      dontSpam = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {//necessary
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (BallsShotCounter == shootXAmntBalls || m_timer.get() > ourTime) {
        return true;
      }
      else {
        return false;
    } 
  }

  
}