/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autons;

import frc.robot.tasks.EmergencyAuton;
import frc.robot.tasks.MoveOffLine;
import frc.robot.tasks.ShootFromAutonLine;
import frc.robot.tasks.TaskBase;
import frc.robot.tasks.ZeroHoodMotor;

/**
 * Add your docs here.
 */
public class ShootAndMoveBackward extends AutonBase {
    @Override
    public TaskBase[] getTasks() {
        return new TaskBase[]{
                new ZeroHoodMotor(),
                new EmergencyAuton(new ShootFromAutonLine(), 5, new MoveOffLine()),
                new ZeroHoodMotor(),
                new MoveOffLine()
        };
    }
}
