package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import io.github.pseudoresonance.pixy2api.Pixy2;

public class Intake_Subsystem extends SubsystemBase {
    WPI_VictorSPX intakeVictor;

    SmartDashboard smartDashboard;

    double errorDefaultValue = -1.0;

    Solenoid leftPiston;
    Solenoid rightPiston;

    boolean intakeDown = false;
    boolean toggleIntakeDirection = false;

    Pixy2 cartridgepixy;
    Pixy2 intakepixy;

    double intakeVictorOut;
    
    public Intake_Subsystem() {
        intakeVictor = RobotContainer.intakeVictor;
        leftPiston = RobotContainer.leftPneumaticPiston;
        rightPiston = RobotContainer.rightPneumaticPiston;
        cartridgepixy = RobotContainer.cartridgePixy;
        intakepixy = RobotContainer.intakePixy;
        
    }

    @Override
    public void periodic() {
        //getAmmoCount();
        
    }

    public void toggleIntake(boolean wantsOn, boolean reversed) {
        if (wantsOn) {
            intakeBalls(reversed);
        }
        else {
            endIntake();
        }
        
    }

    public void intakeBalls(boolean reversed) {
        if (reversed) {//cartridgepixy.getCCC().getBlocks(false, 1, 1) == 1) { //if storage is full...
            /*try { //may need changes for when balls are added, but the cartridge isn't full
            Thread.sleep((long) Constants.intakeWaitTime);
            }
            catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
            }
            */
            intakeVictor.set(Constants.intakeVictorOutput);
            System.out.println("Intake Out is :" + intakeVictorOut);
            
        }
        else {
            intakeVictor.set(-Constants.intakeVictorOutput);
           
        }
    }

    //All code below is outdated...
    /*
    public double getAmmoCount() {
        //smartDashboard.putNumber("Title", double number);
        refreshAmmoCount();
        return SmartDashboard.getNumber("Ball Count", errorDefaultValue);
    }

    public void refreshAmmoCount() {
        boolean catchError = false;

        if (SmartDashboard.getBoolean("Add 1 Ball", catchError) == true) {
            SmartDashboard.putNumber("Ball Count", (SmartDashboard.getNumber("Ball Count", errorDefaultValue) + 1.0));
            SmartDashboard.putBoolean("Add 1 Ball", false);
        }
        else if (SmartDashboard.getBoolean("Subtract 1 Ball", catchError) == true) {
            SmartDashboard.putNumber("Ball Count", (SmartDashboard.getNumber("Ball Count", errorDefaultValue) - 1.0));
            SmartDashboard.putBoolean("Subtract 1 Ball", false);
        }
    } */

    public void endIntake() {
        intakeVictor.set(0);
    }

    public void autoIntake() {
        //cartridgepixy.getCCC().getBlocks()...
        //need to add a system that will orient the shooter towards the balls for autonomous
    }
}