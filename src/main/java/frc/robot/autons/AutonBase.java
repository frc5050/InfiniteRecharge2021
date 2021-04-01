/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autons;

import frc.robot.Robot;
import frc.robot.tasks.TaskBase;

/**
 * Add your docs here.
 */
public abstract class AutonBase {
    protected TaskBase[] tasks;
    protected int currentTask;

    public AutonBase() {
        Robot.addAuton(this);
    }

    public String getName() {
        return this.getClass().getSimpleName();
    }

    public boolean isDefault() {
        return false;
    }

    public abstract TaskBase[] getTasks();

    public void start() {
        currentTask = 0;
        tasks = getTasks();

        if (tasks.length > 0) tasks[0].start();
       // System.out.println(tasks[0].getClass().getName() + " started");
    }

    public void periodic() {
        //if (currentTask < tasks.length) System.out.println(tasks[currentTask].getClass().getName() + " periodic");
        if (currentTask < tasks.length && tasks[currentTask].periodic()) {
            tasks[currentTask].done();
            //System.out.println(tasks[currentTask].getClass().getName() + " finished");
            currentTask += 1;
            if (currentTask < tasks.length) {
                tasks[currentTask].start();
              //  System.out.println(tasks[currentTask].getClass().getName() + " started");
            }
        }
    }

    public void done() {
        if (currentTask < tasks.length) tasks[currentTask].done();
    }
}
