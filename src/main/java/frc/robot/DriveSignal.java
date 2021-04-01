/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ControlType;

/**
 * Add your docs here.
 */
public class DriveSignal {
    private double leftPower;
    private double rightPower;
    private ControlType controlType;

    public DriveSignal(double leftPower, double rightPower) {
        this(leftPower, rightPower, ControlType.kDutyCycle);
    }

    public DriveSignal(double leftPower, double rightPower, ControlType controlType) {
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        this.controlType = controlType;
    }

    public double getLeftPower() {
        return leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }

    public ControlType getControlType() {
        return controlType;
    }
}
