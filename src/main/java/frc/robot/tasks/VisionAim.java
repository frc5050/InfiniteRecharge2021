/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tasks;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

/**
 * Add your docs here.
 */
public class VisionAim implements TaskBase {
    private final DriveTrain driveTrain;
    private final Limelight limelight;

    public VisionAim(){
        driveTrain = DriveTrain.getInstance();
        limelight = Limelight.getInstance();
    }

    @Override
    public void start() {
        limelight.setLED(true);
    }

    @Override
    public boolean periodic() {
        driveTrain.visionLoop();
        return driveTrain.isAimedAtTarget();
    }

    @Override
    public void done() {
        limelight.setLED(false);
    }
}
