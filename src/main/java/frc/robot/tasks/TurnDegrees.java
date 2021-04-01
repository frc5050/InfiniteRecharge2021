/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tasks;

import frc.robot.subsystems.DriveTrain;

/**
 * Add your docs here.
 */
public class TurnDegrees implements TaskBase {
    private double degrees;
    private final DriveTrain driveTrain = DriveTrain.getInstance();

    public TurnDegrees(double degrees) {
        this.degrees = Math.abs(degrees / 360) * 34.25;
    }

    @Override
    public void start() {
        if (degrees >= 0) {
            driveTrain.setMotorPower(-0.25, 0.25);
        } else {
            driveTrain.setMotorPower(0.25, -0.25);
        }
    }

    @Override
    public boolean periodic() {
        return driveTrain.leftEncoderFront.getPosition() >= 25;
    }

    @Override
    public void done() {
      //  System.out.println(driveTrain.getAngle());
        driveTrain.setMotorPower(0, 0);
    }
}
