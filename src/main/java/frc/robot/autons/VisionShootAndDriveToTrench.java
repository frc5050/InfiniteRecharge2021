/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autons;

import frc.robot.tasks.DeployIntakeTrench;
import frc.robot.tasks.DriveDistance;
import frc.robot.tasks.EmergencyAuton;
import frc.robot.tasks.IntakeComeBack;
import frc.robot.tasks.MoveOffLine;
import frc.robot.tasks.NavXTurnDegrees;
import frc.robot.tasks.ParallelTask;
import frc.robot.tasks.RevUp;
import frc.robot.tasks.Shoot;
import frc.robot.tasks.TaskBase;
import frc.robot.tasks.VisionAim;
import frc.robot.tasks.ZeroHoodMotor;

import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class VisionShootAndDriveToTrench extends AutonBase {
    @Override
    public TaskBase[] getTasks() {
        return new TaskBase[]{
                new ZeroHoodMotor(),
                new VisionAim(),
                new EmergencyAuton(new Shoot(AUTOLINE_ORDER_66, AUTOLINE_DISTURBING_FORCE), 5, new MoveOffLine()),
                new DeployIntakeTrench(),
                new DriveDistance(-120 * 25.4, 0.5),
                new NavXTurnDegrees(14),
                new ParallelTask(new DriveDistance(-72 * 25.4, 0.4), new ZeroHoodMotor()),
                new IntakeComeBack(),
                new NavXTurnDegrees(-5),
                new ParallelTask(new DriveDistance(130 * 25.4, -0.675), new RevUp(TRENCH_ORDER_66, TRENCH_POSITION)),
                new VisionAim(),
                new Shoot(TRENCH_ORDER_66, TRENCH_POSITION),
                //new DriveDistance(120 * 25.4, true),
                //new DriveDistance(-72 * 25.4),
        };
    }
}
