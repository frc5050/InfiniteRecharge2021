// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tasks;

import java.util.function.Supplier;

/** Add your docs here. */
public class IfTask implements TaskBase {
    Supplier<Boolean> condition;
    TaskBase trueTask;
    TaskBase falseTask;
    TaskBase chosenTask;


    public IfTask(Supplier<Boolean> condition, TaskBase trueTask, TaskBase falseTask){
        this.condition = condition;
        this.trueTask = trueTask;
        this.falseTask = falseTask;
        

    }

    @Override
    public void start() {
        if (condition.get()){
            chosenTask = trueTask;
        }else {
            chosenTask = falseTask;
        }
        chosenTask.start();

    }

    @Override
    public boolean periodic() {
        return chosenTask.periodic();
    }

    @Override
    public void done() {
       chosenTask.done();

    }
}
