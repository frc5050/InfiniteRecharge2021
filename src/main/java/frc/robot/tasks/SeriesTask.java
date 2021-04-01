// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tasks;

/** Add your docs here. */
public class SeriesTask implements TaskBase{
    private TaskBase[] tasks;
    private int currentTask;

    public SeriesTask(TaskBase[] tasks){
        this.tasks = tasks;
    }

    public void start() {
        currentTask = 0;
        if(tasks.length > 0)
            tasks[0].start();
       // System.out.println(tasks[0].getClass().getName() + " started");
    }

    public boolean periodic() {
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
        return currentTask >= tasks.length; 
    }

    public void done() {
        if (currentTask < tasks.length) tasks[currentTask].done();
    }
}
