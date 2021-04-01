/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tasks;

import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class EmergencyAuton implements TaskBase {
    private TaskBase taskBeingRun;
    private double timeAlloted;
    private TaskBase alternateTask;
    private boolean isCancelled;
    private boolean callbackFinished;
    private Timer timer;

    public EmergencyAuton(TaskBase taskBeingRun, double timeAlloted, TaskBase alternateTask) {
        this.taskBeingRun = taskBeingRun;
        this.timeAlloted = timeAlloted;
        this.alternateTask = alternateTask;

    }

    @Override
    public void start() {
        timer = new Timer();
        timer.reset();
        timer.start();
        taskBeingRun.start();
        isCancelled = false;
        callbackFinished = false;
    }

    @Override
    public boolean periodic() {
        if (timer.get() <= timeAlloted) {
            return taskBeingRun.periodic();
        } else if (!isCancelled) {
            taskBeingRun.done();
            alternateTask.start();
            isCancelled = true;
        } else if (!callbackFinished) {
            timer.stop();
            callbackFinished = alternateTask.periodic();
            if (callbackFinished) {
                alternateTask.done();
            }
        }
        return false;
    }

    @Override
    public void done() {
        if (!isCancelled) {
            taskBeingRun.done();
        } else if (!callbackFinished) {
            alternateTask.done();
        }
    }

}
