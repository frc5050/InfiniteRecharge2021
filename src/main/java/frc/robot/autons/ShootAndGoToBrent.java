/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autons;

import frc.robot.tasks.DeployIntakeTrench;
import frc.robot.tasks.DriveDistance;
import frc.robot.tasks.IntakeComeBack;
import frc.robot.tasks.NavXTurnDegrees;
import frc.robot.tasks.ShootFromAutonLine;
import frc.robot.tasks.TaskBase;
import frc.robot.tasks.ZeroHoodMotor;

/**
 * Add your docs here.
 */
public class ShootAndGoToBrent extends AutonBase {
    @Override
    public TaskBase[] getTasks() {
        return new TaskBase[]{
                new ZeroHoodMotor(),
                new ShootFromAutonLine(),
                new DeployIntakeTrench(),
                new DriveDistance(-78 * 25.4, 0.5),
                new NavXTurnDegrees(22.5),
                new IntakeComeBack(),
                //new DriveDistance(72 * 25.4, true, Constants.AUTOLINE_ORDER_66, Constants.AUTOLINE_DISTURBING_FORCE, -0.5),
                //new DriveDistance(-, power)


        };
    }
}
