package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousAutoShoot_Command;
import frc.robot.commands.AutoDriveStraight_Command;
import frc.robot.commands.AutoShoot_Command;
import frc.robot.commands.OffIntake_Command;
import frc.robot.commands.OnIntake_Command;
import frc.robot.commands.UpwardStorage_Command;
import frc.robot.subsystems.DriveBase_Subsystem;

public class DriveBack_Command extends SequentialCommandGroup {

    public DriveBack_Command() {
        addCommands(
            new AutoDriveStraight_Command(0.5, -1.0),
            new OnIntake_Command()
            );
    }

    
}
