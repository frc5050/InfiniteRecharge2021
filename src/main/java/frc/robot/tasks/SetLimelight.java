// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tasks;

import frc.robot.subsystems.Limelight;

/** Add your docs here. */
public class SetLimelight implements TaskBase {
    private boolean enabled;

    public SetLimelight(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public void start() {
        Limelight.getInstance().setLED(enabled);
        

    }

    @Override
    public boolean periodic() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub

    }


}
