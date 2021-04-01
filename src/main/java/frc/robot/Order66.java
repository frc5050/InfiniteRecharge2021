/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ControlType;

/**
 * Add your docs here.
 */
public class Order66 {
    public ControlType controlType;
    public double demand;

    public Order66(ControlType controlType, double demand) {
        this.controlType = controlType;
        this.demand = demand;
    }


}
