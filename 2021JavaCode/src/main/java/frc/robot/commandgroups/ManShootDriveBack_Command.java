package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousAutoShoot_Command;
import frc.robot.commands.AutoDriveStraight_Command;
import frc.robot.commands.AutoIntakeOn_Command;
import frc.robot.commands.AutoShoot_Command;
import frc.robot.commands.AutonomousManShoot_Command;
import frc.robot.commands.OffIntake_Command;
import frc.robot.commands.OnIntake_Command;
import frc.robot.commands.StahpTheShoot_Command;
import frc.robot.commands.StopShootStorage_Command;
import frc.robot.commands.UpwardStorage_Command;
import frc.robot.subsystems.DriveBase_Subsystem;

public class ManShootDriveBack_Command extends SequentialCommandGroup {

    public ManShootDriveBack_Command() {
        addCommands(
            new AutoIntakeOn_Command(0.5),
            new UpwardStorage_Command(),
            new AutonomousManShoot_Command(3, 10.0),
            new StahpTheShoot_Command(),
            new StopShootStorage_Command(),
            new AutoDriveStraight_Command(0.5, -1.0)
            );
    }

    
}
