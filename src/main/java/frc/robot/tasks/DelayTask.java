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
public class DelayTask implements TaskBase{
    private Timer timer;
    private double delay;
    public DelayTask(double delay){
        this.delay = delay;
        timer = new Timer();

    }

    @Override
    public void start() {
        timer.reset();
        timer.start();
    }

    @Override
    public boolean periodic() {
       return timer.get() >= delay;
    }

    @Override
    public void done() {
        timer.stop();

    }
}
