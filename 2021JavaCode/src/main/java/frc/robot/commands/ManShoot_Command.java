package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter_Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ManShoot_Command extends CommandBase {

    public ManShoot_Command() {
        addRequirements(RobotContainer.shooter_Subsystem);
        addRequirements(RobotContainer.driveBase_Subsystem);
        addRequirements(RobotContainer.storage_Subsystem); //necessary?
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      System.out.println("Shooter Be shootin");
      //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      //table.getEntry("pipeline").setNumber(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.shooter_Subsystem.shoot(Constants.shooterAngle, false, true, false) == true) {//ready to shoot {
      RobotContainer.storage_Subsystem.store(true, true, false, false, false);
    }
    else {
      RobotContainer.storage_Subsystem.store(true, false, false, false, false);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {//necessary
    //RobotContainer.shooter_Subsystem.endShoot();
    //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //table.getEntry("pipeline").setNumber(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  
}