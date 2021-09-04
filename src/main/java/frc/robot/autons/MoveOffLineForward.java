// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

import frc.robot.tasks.DriveDistance;
import frc.robot.tasks.TaskBase;

/** Add your docs here. */
public class MoveOffLineForward extends AutonBase{

	@Override
	public TaskBase[] getTasks() {
		return new TaskBase[]{
            new DriveDistance(-30 * 25.4, -0.3)

        };
	}
  
}
