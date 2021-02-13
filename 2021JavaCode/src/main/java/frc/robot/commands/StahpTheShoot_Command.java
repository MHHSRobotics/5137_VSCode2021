package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class StahpTheShoot_Command extends CommandBase {

    public StahpTheShoot_Command() {
        addRequirements(RobotContainer.shooter_Subsystem);
        addRequirements(RobotContainer.driveBase_Subsystem);
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      System.out.println("Shooter Be shootin");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter_Subsystem.endShoot();
    //RobotContainer.storage_Subsystem.store(true, false, false, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {//necessary

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  
}