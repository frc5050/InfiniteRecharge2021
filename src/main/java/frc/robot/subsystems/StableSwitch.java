/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class StableSwitch {
    private DigitalInput digitalInput;
    private int counter = 0;

    public StableSwitch(int portNumber) {
        digitalInput = new DigitalInput(portNumber);
    }

    public void periodic() {
        if (digitalInput.get()) {
            counter += 1;
        } else {
            counter = 0;
        }
    }

    public boolean get() {
        return counter >= 3;
    }
}


