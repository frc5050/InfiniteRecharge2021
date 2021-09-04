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
import frc.robot.tasks.FindPath;
import frc.robot.tasks.IntakeComeBack;
import frc.robot.tasks.MoveOffLine;
import frc.robot.tasks.NavXTurnDegrees;
import frc.robot.tasks.ParallelTask;
import frc.robot.tasks.RevUp;
import frc.robot.tasks.SetLimelight;
import frc.robot.tasks.Shoot;
import frc.robot.tasks.TaskBase;
import frc.robot.tasks.VisionAim;
import frc.robot.tasks.ZeroHoodMotor;

import static frc.robot.Constants.*;

import java.io.FileNotFoundException;

import frc.robot.subsystems.PathLoader;

/**
 * Add your docs here.
 */
public class SixBallTrench extends AutonBase {
    @Override
    public TaskBase[] getTasks() {
        try {
            return new TaskBase[] { new ZeroHoodMotor(),
                    new SetLimelight(true),
                    new EmergencyAuton(new Shoot(AUTOLINE_ORDER_66, AUTOLINE_DISTURBING_FORCE), 5, new MoveOffLine()),
                    new DeployIntakeTrench(),
                    
                    new FindPath(PathLoader.getInstance().getPath("TrenchPath.wpilib.json")),
                    new FindPath(PathLoader.getInstance().getPath("TrenchToAutoLine.wpilib.json")),
                    new VisionAim(),
                    new Shoot(AUTOLINE_ORDER_66, AUTOLINE_DISTURBING_FORCE),
                    new SetLimelight(false),

                    new IntakeComeBack()
                 };     
        } catch (FileNotFoundException e) {
            e.printStackTrace(); 
            return new TaskBase[0];
           
        }
    }
}
