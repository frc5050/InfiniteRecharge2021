// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

import java.io.FileNotFoundException;

import frc.robot.subsystems.PathLoader;
import frc.robot.tasks.FindPath;
import frc.robot.tasks.TaskBase;

/** Add your docs here. */
public class BouncePath extends AutonBase {

    @Override
    public TaskBase[] getTasks() {
        try{
            return new TaskBase[] {
                new FindPath(PathLoader.getInstance().getPath("BounceStart1.wpilib.json")),
                new FindPath(PathLoader.getInstance().getPath("BounceReverse2.wpilib.json")),
                new FindPath(PathLoader.getInstance().getPath("BounceForward3.wpilib.json")),
                new FindPath(PathLoader.getInstance().getPath("BounceEnd4.wpilib.json"))
            };
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        
        return new TaskBase[0];
    }

}
