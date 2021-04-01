/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystem;

import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {
    private NetworkTable limelightNt;
    private static Limelight instance = null;

    private Limelight() {
        limelightNt = NetworkTableInstance.getDefault().getTable("limelight");
        setLED(false);
    }

    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }

        return instance;
    }

    public void setLED(boolean isOn) {
        if (isOn) {
            limelightNt.getEntry("ledMode").setNumber(3);
            limelightNt.getEntry("canMode").setNumber(0);
        } else {
            limelightNt.getEntry("ledMode").setNumber(1);
            limelightNt.getEntry("canMode").setNumber(1);
        }
    }


    public double getDistance() {
        double a2 = limelightNt.getEntry("ty").getDouble(0.0);
        double distance = (H2CENTER - H1) / Math.tan((A1 + a2) * Math.PI / 180);
       // System.out.println(distance);
       // SmartDashboard.putNumber("Horizontal Distance To Target", distance);
        return distance;
    }

    public double getAngleToTarget() {
        return limelightNt.getEntry("tx").getDouble(0);
    }

    public boolean seesTarget() {
        return limelightNt.getEntry("tv").getDouble(0) == 1.0;
    }


}
