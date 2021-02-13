package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DownwardStorage_Command extends CommandBase {
    
    public DownwardStorage_Command() {
        addRequirements(RobotContainer.storage_Subsystem);
    }

    @Override
  public void initialize() {
      System.out.println("Storage Running...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.storage_Subsystem.store(true, false, true, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; //may need to change, may want it to end naitively some other way
  }
}