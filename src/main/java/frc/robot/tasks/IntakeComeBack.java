/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tasks;

import frc.robot.subsystems.Blinky;

/**
 * Add your docs here.
 */
public class IntakeComeBack implements TaskBase {

    @Override
    public void start() {
        Blinky.getInstance().wantToIntake = false;
    }

    @Override
    public boolean periodic() {
        return true;
    }

    @Override
    public void done() {
    }

}
