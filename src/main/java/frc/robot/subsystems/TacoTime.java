/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Controllers;
import frc.robot.Subsystem;

import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class TacoTime extends Subsystem {
    Solenoid climbSolenoid;
    TalonSRX winchMotor;
    TalonSRX climbHeightMotor;
    CANSparkMax balanceMotor;
    private static TacoTime instance;
    private final Controllers controllers;
    private final ColorSensor colorSensor;

    

    private TacoTime() {
        climbSolenoid = new Solenoid(PCM, CLIMB_SOLENOID);
        winchMotor = new TalonSRX(WINCH_MOTOR);
        climbHeightMotor = new TalonSRX(CLIMB_HEIGHT_MOTOR);
        balanceMotor = new CANSparkMax(COLOR_WHEEL_BALANCE_MOTOR, MotorType.kBrushless);

        controllers = Controllers.getInstance();
        colorSensor = ColorSensor.getInstance();
    }

    public static TacoTime getInstance(){
        if (instance == null){
            instance = new TacoTime();
        }
        return instance;
    }

    @Override
    public void generalInit() {
        climbSolenoid.set(false);
        winchMotor.set(ControlMode.PercentOutput, 0);
        climbHeightMotor.set(ControlMode.PercentOutput, 0);
        balanceMotor.set(0);
    }

    @Override
    public void teleopPeriodic() {
        climbSolenoid.set(controllers.xHeld());


        if (controllers.rightTriggerHeld() >= 0.02) {
            winchMotor.set(ControlMode.PercentOutput, controllers.rightTriggerHeld());
            climbHeightMotor.set(ControlMode.PercentOutput, 0.5);
        } else {
            winchMotor.set(ControlMode.PercentOutput, 0);
        }


        if (controllers.leftBumperHeld()) {
            climbHeightMotor.set(ControlMode.PercentOutput, 1.0);
        } else if (controllers.rightBumperHeld()) {
            climbHeightMotor.set(ControlMode.PercentOutput, -1.0);
        } else {
            climbHeightMotor.set(ControlMode.PercentOutput, 0.0);
        }


        if (controllers.getGamepadX(Hand.kLeft) >= 0.07) {
            balanceMotor.set(controllers.getGamepadX(Hand.kLeft));
            colorSensor.colorSpinManualMode = true;
        } else if (controllers.getGamepadX(Hand.kLeft) <= -0.07) {
            balanceMotor.set(controllers.getGamepadX(Hand.kLeft));
            colorSensor.colorSpinManualMode = true;
        } else if (colorSensor.colorSpinManualMode) {
            balanceMotor.set(0);

        }

    }
}
