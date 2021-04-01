// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

import java.io.FileNotFoundException;

import frc.robot.subsystems.Blinky;
import frc.robot.subsystems.PathLoader;
import frc.robot.tasks.DeployIntakeTrench;
import frc.robot.tasks.FindPath;
import frc.robot.tasks.IfTask;
import frc.robot.tasks.IntakeComeBack;
import frc.robot.tasks.TaskBase;

/** Add your docs here. */
public class GalacticSearchB extends AutonBase {

    @Override
    public TaskBase[] getTasks() {

        try {
            return new TaskBase[] { 
                new DeployIntakeTrench(),
                new FindPath(PathLoader.getInstance().getPath("GalacticSearchBStart.wpilib.json")),
                new IfTask(()-> Blinky.getInstance().blinkyEmpty(), 
                    new FindPath(PathLoader.getInstance().getPath("GalacticSearchBBlue.wpilib.json")), 
                    new FindPath(PathLoader.getInstance().getPath("GalacticSearchBRed.wpilib.json"))),
                new IntakeComeBack()

            };
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        return new TaskBase [0];
    }
}
