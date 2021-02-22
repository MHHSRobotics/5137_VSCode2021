/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CargoBox_Subsystem extends SubsystemBase {

  public static DoubleSolenoid cargoBoxPiston = RobotMap.cargoBoxPiston;

  public static DoubleSolenoid.Value getBoxStatus() {
    return cargoBoxPiston.get();
  }

  public static void openBox() {
    cargoBoxPiston.set(Value.kForward);
  }

  public static void closeBox() {
    cargoBoxPiston.set(Value.kReverse);
  }

  public static void stop() {
    cargoBoxPiston.set(Value.kOff);
  }
  
}
