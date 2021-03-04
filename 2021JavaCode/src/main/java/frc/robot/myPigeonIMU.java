// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

/** Add your docs here. */
public class myPigeonIMU extends PigeonIMU {
    PigeonIMU m_PigeonIMU;

    public myPigeonIMU(int deviceNumber) {
        super(deviceNumber);
        m_PigeonIMU = new PigeonIMU(deviceNumber);
        // TODO Auto-generated constructor stub
    }

    public Rotation2d getRotation2d() {
        double [] ypr = new double[3];

        m_PigeonIMU.getYawPitchRoll(ypr);
        return Rotation2d.fromDegrees(-ypr[0]);
    }

    public void reset() {
        m_PigeonIMU.setYaw(0);
    }
}
