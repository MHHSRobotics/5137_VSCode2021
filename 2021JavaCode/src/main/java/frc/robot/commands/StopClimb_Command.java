package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class StopClimb_Command extends CommandBase {
    public StopClimb_Command() {
    // Add necessary subsystem (what subsystems are needed)
    addRequirements(RobotContainer.climb_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Climbing Up...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.climb_Subsystem.stop();
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