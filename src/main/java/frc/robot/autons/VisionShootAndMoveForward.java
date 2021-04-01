/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autons;

import frc.robot.tasks.DriveDistance;
import frc.robot.tasks.ShootFromAutonLine;
import frc.robot.tasks.TaskBase;
import frc.robot.tasks.VisionAim;
import frc.robot.tasks.ZeroHoodMotor;

/**
 * Add your docs here.
 */
public class VisionShootAndMoveForward extends AutonBase {
    @Override
    public TaskBase[] getTasks() {
        return new TaskBase[]{
            new ZeroHoodMotor(),
            new VisionAim(),
            new ShootFromAutonLine(),
            new DriveDistance(48 * 25.4, -0.4),
        };
        
    }

}
