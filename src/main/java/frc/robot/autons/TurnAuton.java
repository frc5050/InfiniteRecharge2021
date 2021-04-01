/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autons;

import frc.robot.tasks.NavXTurnDegrees;
import frc.robot.tasks.TaskBase;

/**
 * Add your docs here.
 */
public class TurnAuton extends AutonBase {
    @Override
    public TaskBase[] getTasks() {
        return new TaskBase[]{
                new NavXTurnDegrees(-20)
        };

    }
}
