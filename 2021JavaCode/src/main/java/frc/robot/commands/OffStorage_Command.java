package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class OffStorage_Command extends CommandBase {

    public OffStorage_Command() {
        addRequirements(RobotContainer.storage_Subsystem);
    }

    @Override
  public void initialize() {
      System.out.println("Storage Overriden...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.storage_Subsystem.store(false, false, false, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*RobotContainer.lstorageVictor.set(0);
    RobotContainer.rstorageVictor.set(0); */ //may need to change... 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; 
  }
}