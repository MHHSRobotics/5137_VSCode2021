package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ControlPanel_Subsystem extends SubsystemBase {
    WPI_TalonSRX controlPanelTalon;

    ColorSensorV3 colorSensor;
    Color fieldsSensedColor;
    Color FMSColor;

    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public ControlPanel_Subsystem() {
        controlPanelTalon = RobotContainer.controlPanelTalon;
        colorSensor = RobotContainer.colorSensor;
        System.out.println("ControlPanel Going");
    }

    @Override
    public void periodic() {
        getColorSignal();
        System.out.println("Control Panel Victor Output: " + controlPanelTalon.getMotorOutputPercent());
    }


    public Color getColorSignal() { 

        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
            case 'B':
                // Blue case code
                return kBlueTarget;

            case 'G':
                // Green case code
                return kGreenTarget;

            case 'R':
                // Red case code
                return kRedTarget;

            case 'Y':
                // Yellow case code
                return kYellowTarget;
                
            default:
                // This is corrupt data
                return Color.kBlack;
            }
        } else {
            // Code for no data received yet
            return Color.kWhite;
        }
    }//need to send output of determined color to smartdashboard

    public void controlPosition(double motorRpm) { //calculates the amount of time the motor should run at a given velocity
        fieldsSensedColor = guessCurrentColorOnSensor();
        FMSColor = getColorSignal();

        spinMotor(calcNumberRotations(fieldsSensedColor, FMSColor), convertRPMPercent(motorRpm), motorRpm);

    }

    public void controlRotation(double motorRpm) { //calculates the amount of time the motor should run to complete rotation control
        spinMotor(Constants.SpinWheel, convertRPMPercent(motorRpm), motorRpm);
    }

    public Color guessCurrentColorOnSensor() {
        Color colorSensorColorRead = colorSensor.getColor();
        if (colorSensorColorRead.blue > Constants.blueTargetMin) {
            return kBlueTarget;
        }
        else if (colorSensorColorRead.red > Constants.redTargetMin) {
            return kRedTarget;
        }
        else if (colorSensorColorRead.green > Constants.greenTargetMin) {
            return kGreenTarget;
        }
        else if (colorSensorColorRead.green > Constants.yellowGreenTargetMin && colorSensorColorRead.red < Constants.yellowRedTargetMax) {
            return kYellowTarget;
        }
        else {
            System.out.println("BLACK AHHHGH");
            return Color.kBlack;
        }
    }

    public double calcNumberRotations(Color kColorInput, Color kColorFMS) { 
        //takes in parameters of field's sensed color, and the fields desired color

        //Color wheel colors in order (ccw): red, green, blue, yellow
        if (kColorInput == kColorFMS) {
            return 0.0;
        }
        else {
            if ((kColorInput == kRedTarget && kColorFMS == kGreenTarget) || 
            (kColorInput == kGreenTarget && kColorFMS == kBlueTarget) || 
            (kColorInput == kBlueTarget && kColorFMS == kYellowTarget) ||
            (kColorInput == kYellowTarget && kColorFMS == kRedTarget)) {
                return 0.125;
            }
            if ((kColorInput == kRedTarget && kColorFMS == kBlueTarget) ||
            (kColorInput == kBlueTarget && kColorFMS == kRedTarget) ||
            (kColorInput == kGreenTarget && kColorFMS == kYellowTarget) ||
            (kColorInput == kYellowTarget && kColorFMS == kGreenTarget)) {
                return 0.25;
            }
            if ((kColorInput == kRedTarget && kColorFMS == kYellowTarget) ||
            (kColorInput == kGreenTarget && kColorFMS == kRedTarget) ||
            (kColorInput == kBlueTarget && kColorFMS == kGreenTarget) ||
            (kColorInput == kYellowTarget && kColorFMS == kBlueTarget)) {
                return -0.125;
            }
        }
        return 0.0;
    }
    public void spinMotor(double numberRotationsBig, double percentOut, double motorRpm) {
        //Note, currently, using 4 in. wheels on the control panel thing will make the ratio of curcumference 1:8, meaning one motor 
        //output spin means EXACTLY one color change on the wheel
        //double distanceTravel = 0.0;
        double waitTime = 0.0;
        double numberRotationsSmall = 0.0;

        //0.125 as a rotation of the wheel = 1 spin of the actual controlled motor
        numberRotationsSmall = numberRotationsBig * 8.0; //converts return value into # rotations of gear wheel


        //Finds circumference
        //distanceTravel = numberRotationsSmall * Constants.CPWheelCircum; //circuference of wheel is 8PI. Dist travel is in in.
         
        //Finds time to set motors for (in mms)
        waitTime = Math.abs(1000.0 * (100.0 * numberRotationsSmall / motorRpm));

        if (numberRotationsBig < 0.0) { //if motor needs to change direction (i.e. big wheel needs to go left)
            controlPanelTalon.set(-percentOut);
        }
        else {
            controlPanelTalon.set(percentOut);
        }
        
        try {
            Thread.sleep((long) waitTime);
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        controlPanelTalon.set(0);

        //ex. controlPanelVictor.set(30);
        
    }

    public void senseColor() {
        Color detectedColor = colorSensor.getColor();
        double IR = colorSensor.getIR();
    
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);
      }

    public double convertRPMPercent(double rpm) {
        double returnThing = (rpm * Constants.rpmToPercentOutFactor)/100;
        System.out.println("Percent Output is : " + returnThing);
        return returnThing;
    }
}
