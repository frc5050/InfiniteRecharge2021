/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tasks;

import java.util.ArrayList;
import java.util.List;

public class ParallelTask implements TaskBase {
    private List<TaskBase> unfinishedTasks;

    /**
     * Runs multiple tasks at the same time, and finishes when all tasks are done
     *
     * @param tasks tasks you want to run in parallel
     */

    public ParallelTask(TaskBase... tasks) {
        unfinishedTasks = new ArrayList<TaskBase>();
        for (TaskBase task : tasks) {
            unfinishedTasks.add(task);
        }
    }

    @Override
    public void start() {
        //Starts all of the tasks
        for (TaskBase task : unfinishedTasks) {
            task.start();
        }
    }

    @Override
    public boolean periodic() {
        for (int i = 0; i < unfinishedTasks.size(); i++) {
            TaskBase task = unfinishedTasks.get(i);
            if (task.periodic()) {
                task.done();
                unfinishedTasks.remove(i);
                i--;
            }
        }
        return unfinishedTasks.isEmpty();
    }

    @Override
    public void done() {
        for (TaskBase task : unfinishedTasks) {
            task.done();
        }
    }
}
