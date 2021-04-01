/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autons;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;

import frc.robot.Order66;
import frc.robot.subsystems.DisturbingForce;
import frc.robot.tasks.DriveDistance;
import frc.robot.tasks.Shoot;
import frc.robot.tasks.TaskBase;
import frc.robot.tasks.VisionAim;
import frc.robot.tasks.ZeroHoodMotor;

/**
 * Add your docs here.
 */
public class VisionShootAndMoveFar extends AutonBase {
    @Override
    public TaskBase[] getTasks() {
        return new TaskBase[]{
            new ZeroHoodMotor(),
            new VisionAim(),
            new Shoot(new Order66(ControlType.kVelocity, 3200), new DisturbingForce(ControlMode.Position, 430000)),
            new DriveDistance(-54 * 25.4, 0.4),
        };
        
    }
}
