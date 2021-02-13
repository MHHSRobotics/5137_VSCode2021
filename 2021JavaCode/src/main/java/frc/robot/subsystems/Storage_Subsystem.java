package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;

public class Storage_Subsystem extends SubsystemBase {

    Pixy2 pixy;
    
    WPI_VictorSPX lstorageVictor;
    WPI_VictorSPX rstorageVictor;

    boolean invertStore;

    public Storage_Subsystem() {
        lstorageVictor = RobotContainer.lstorageVictor;
        rstorageVictor = RobotContainer.rstorageVictor;
        pixy = RobotContainer.cartridgePixy;
    }

    public void store(boolean wantsOn, boolean shooting, boolean reversed, boolean reversedMatters, boolean wantsBottomBeltReversed) {
        
    if (wantsOn) { //l is top, r is bottom belts
        if(shooting) {
            rstorageVictor.set(Constants.storageSpeed);
        }
        else {
            rstorageVictor.set(0);
        }
        if(reversed && reversedMatters) {
            lstorageVictor.set(-Constants.storageSpeed);
        }
        else if (!reversed && reversedMatters) {
            lstorageVictor.set(Constants.storageSpeed);
        }
        else {
            //don't reset the top belt
        }
        if (wantsBottomBeltReversed) {
            rstorageVictor.set(-Constants.storageSpeed);
        }
        
        
    }
    else {
        lstorageVictor.set(0);
        rstorageVictor.set(0);
    }

    }

    public boolean checkIfFull() {

        if (pixy.getCCC().getBlocks(false, 1, 1) == 1) { //check if it sees the target (pink colored paper)
            return true;
        }
        else {
            return false;
        }
    }
}