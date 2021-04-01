/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tasks;

import frc.robot.subsystems.Vader;

import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class ZeroHoodMotor implements TaskBase {
    private final Vader vader = Vader.getInstance();

    @Override
    public void start() {
        vader.setVaderControlMode(ZEROING);
    }

    @Override
    public boolean periodic() {
        return vader.isZeroed();
    }

    @Override
    public void done() {
      //  System.out.println("Finished Zeroing");
        vader.setVaderControlMode(STOP_DISTURBING_FORCE);
    }
}
