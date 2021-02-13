package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter_Subsystem;

public class AutonomousAutoShoot_Command extends CommandBase {

    boolean dontSpam = true;
    int BallsShotCounter = 0;
    int shootXAmntBalls;

    Timer timer;
    double ourTime;

    public AutonomousAutoShoot_Command(double time) {
      ourTime = time;
      timer = new Timer();
        //shootXAmntBalls = shootXBalls;
        addRequirements(RobotContainer.shooter_Subsystem);
        addRequirements(RobotContainer.driveBase_Subsystem);
        addRequirements(RobotContainer.storage_Subsystem); 
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
      System.out.println("Shooter Be shootin");
      //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      //table.getEntry("pipeline").setNumber(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.shooter_Subsystem.shoot(Constants.shooterAngle, true, false, true) == true) {
      RobotContainer.storage_Subsystem.store(true, true, false, false, false);
      /*
      if (dontSpam) {
          BallsShotCounter++;
          dontSpam = false;
      }*/ 
    }
    else {
      RobotContainer.storage_Subsystem.store(true, false, false, false, false);
      //dontSpam = true;
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {//necessary
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { //depends on if PID is dialed in
    if (timer.get() < ourTime) {
      return false;
    }
    else {
      return true;
    }
    /*
    if (BallsShotCounter == shootXAmntBalls) {
        return true;
      }
      else {
        return false;
    } 
    */
  }
}