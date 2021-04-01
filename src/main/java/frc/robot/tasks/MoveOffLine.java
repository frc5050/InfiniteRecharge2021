/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tasks;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveTrain;

/**
 * Add your docs here.
 */
public class MoveOffLine implements TaskBase {
    Timer timer;
    private final DriveTrain driveTrain = DriveTrain.getInstance();

    @Override
    public void start() {
        driveTrain.setMotorPower(0.25, 0.25);
        timer = new Timer();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean periodic() {
        return timer.get() >= 1;
    }

    @Override
    public void done() {
        driveTrain.setMotorPower(0, 0);
    }
}
