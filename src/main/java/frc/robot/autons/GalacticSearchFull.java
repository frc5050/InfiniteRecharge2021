// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

import java.io.FileNotFoundException;

import frc.robot.subsystems.Blinky;
import frc.robot.subsystems.PathLoader;
import frc.robot.tasks.DelayTask;
import frc.robot.tasks.DeployIntakeTrench;
import frc.robot.tasks.DriveDistance;
import frc.robot.tasks.FindPath;
import frc.robot.tasks.IfTask;
import frc.robot.tasks.IntakeComeBack;
import frc.robot.tasks.TaskBase;
import frc.robot.tasks.TurnDegrees;

/** Add your docs here. */
public class GalacticSearchFull extends AutonBase {

    @Override
    public TaskBase[] getTasks() {
        try {
            return new TaskBase []{
        new DeployIntakeTrench(),
        new DriveDistance(-30 * 25.4, 0.25),
        new IfTask(()-> Blinky.getInstance().blinkyEmpty(), 
            new TurnDegrees(90), 
            new FindPath(PathLoader.getInstance().getPath("GSRedA.wpilib.json"))), //normal
        new IfTask(()-> Blinky.getInstance().blinkyFull(),
            new DelayTask(10),
            new IfTask(()-> Blinky.getInstance().blinkyEmpty(),
                new FindPath(PathLoader.getInstance().getPath ("GSBlue.wpilib.json")), //straight line
                new FindPath(PathLoader.getInstance().getPath("GSRedB.wpilib.json")))), //weird circle-thing
        new IntakeComeBack()
            };
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        };
        return new TaskBase [0];
    } 

    public void done (){
        new IntakeComeBack();
    }
}
