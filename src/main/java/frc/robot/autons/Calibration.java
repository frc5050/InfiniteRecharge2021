/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autons;

import java.io.FileNotFoundException;

import frc.robot.subsystems.PathLoader;
import frc.robot.tasks.FindPath;
import frc.robot.tasks.TaskBase;

/**
 * Add your docs here.
 */
public class Calibration extends AutonBase {
    @Override
    public TaskBase[] getTasks(){
        try {
            return new TaskBase[] {
                new FindPath(PathLoader.getInstance().getPath("LessCircle.wpilib.json"))
            };
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        
        return new TaskBase[0];

    }
}
