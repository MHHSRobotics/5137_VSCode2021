package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ControlPanel_Subsystem;

public class ControlPanel_Command extends CommandBase {

  public ControlPanel_Command() {
    // Add necessary subsystem (what subsystems are needed)
    addRequirements(RobotContainer.controlPanel_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (DriverStation.getInstance().getGameSpecificMessage().length() > 0) { //Stage 2 automatic method
      RobotContainer.controlPanel_Subsystem.controlPosition(Constants.cpRPMPOS);
    }
    else { //Stage 1 automatic method
      RobotContainer.controlPanel_Subsystem.controlRotation(Constants.cpRPMROT);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {//necessary
    RobotContainer.controlPanelTalon.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}
