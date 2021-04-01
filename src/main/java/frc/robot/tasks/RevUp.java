/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tasks;

import frc.robot.Order66;
import frc.robot.subsystems.DeathStar;
import frc.robot.subsystems.DisturbingForce;
import frc.robot.subsystems.Vader;

/**
 * Add your docs here.
 */
public class RevUp implements TaskBase {
    Order66 order66;
    DisturbingForce disturbingForce;

    public RevUp(Order66 order66, DisturbingForce disturbingForce) {
        this.order66 = order66;
        this.disturbingForce = disturbingForce;
    }

    @Override
    public void start() {
        DeathStar.getInstance().setOrder66(order66);
        Vader.getInstance().setVaderControlMode(disturbingForce);
    }

    @Override
    public boolean periodic() {
        return true;
    }

    @Override
    public void done() {
    }
}
