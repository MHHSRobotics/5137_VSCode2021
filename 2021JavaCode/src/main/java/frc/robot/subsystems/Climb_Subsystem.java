package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climb_Subsystem extends SubsystemBase {

    WPI_TalonSRX climbTalon;

    DigitalInput limitSwitchUpper;
    DigitalInput limitSwitchLower;

    public boolean allowed;
    
    public Climb_Subsystem() {
        limitSwitchUpper = RobotContainer.LimitSwitchUpper;
        limitSwitchLower = RobotContainer.LimitSwitchLower;
        climbTalon = RobotContainer.climbTalon;
    }

    public void periodic() {
        if (limitSwitchLower.get() && allowed) { //if the lower limit switch isn't pressed (naitive)
            climbTalon.set(0.1);
        }
        else {
            climbTalon.set(0);
        }
    }

    public void goUp() { //true (1) is not pressed, false is pressed
        System.out.println("UPPER SWITCH IS GOING UP:" + limitSwitchUpper.get());
        if (limitSwitchUpper.get()) { //ready to climb
            climbTalon.set(-1.0);
        }        
        else {//limitSwitch is good
            climbTalon.set(0);
        }
    }

    public void goDown() {
        System.out.println("Lower switch is going down: " + limitSwitchLower.get());
        if (limitSwitchLower.get()) {
            climbTalon.set(1.0);
        }
        else { //limitSwitch is good
            climbTalon.set(0);
        }
    }

    public void stop() {
        climbTalon.set(0);
    }
}