/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Controllers;
import frc.robot.Subsystem;

import static frc.robot.Constants.*;

/**
 * Add your docs here..
 */
public class Blinky extends Subsystem {
    private DigitalInput[] irSensors;
    private TalonSRX[] irMotors;
    private boolean shooting;
    private CANSparkMax intakeMotor;
    private DoubleSolenoid intakeDeploy;
    public boolean wantToShoot;
    public boolean wantToIntake;
    public boolean blinkyBackwards;
    public boolean intakeBackwards;
    private static Blinky instance;
    private final Controllers controllers;
    private Blinky() {
        // intake solenoids
        intakeDeploy = new DoubleSolenoid(PCM, INTAKE_DOWN, INTAKE_UP);


        // makes an array of all the IR sensors
        irSensors = new DigitalInput[5];
        irSensors[0] = new DigitalInput(IR_SENSOR_1);
        irSensors[1] = new DigitalInput(IR_SENSOR_2);
        irSensors[2] = new DigitalInput(IR_SENSOR_3);
        irSensors[3] = new DigitalInput(IR_SENSOR_4);
        irSensors[4] = new DigitalInput(IR_SENSOR_5);

        // makes an array of all the motors in Blinky
        intakeMotor = new CANSparkMax(INTAKE_MOTOR, MotorType.kBrushless);
        intakeMotor.setInverted(false);
        intakeMotor.enableVoltageCompensation(11);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.burnFlash();
        irMotors = new TalonSRX[4];
        irMotors[0] = new TalonSRX(IR_MOTOR_2);
        irMotors[1] = new TalonSRX(IR_MOTOR_3);
        irMotors[2] = new TalonSRX(IR_MOTOR_4);
        irMotors[3] = new TalonSRX(IR_MOTOR_5);
        for (int i = 0; i < 4; i++) {
            irMotors[i].enableVoltageCompensation(true);
            irMotors[i].configVoltageCompSaturation(11);
            irMotors[i].setNeutralMode(NeutralMode.Brake);
        }

        controllers = Controllers.getInstance();
    }

    public static Blinky getInstance(){
        if (instance == null){
            instance = new Blinky();
        }
        return instance;
    }

    @Override
    public void generalInit() {
        intakeMotor.setInverted(false);
        blinkyBackwards = false;
        intakeBackwards = false;
        wantToIntake = false;
        wantToShoot = false;

        intakeMotor.set(0);
        for (int i = 0; i < 4; i++) {
            irMotors[i].set(ControlMode.PercentOutput, 0);
        }
    }

    @Override
    public void generalPeriodic() {
        intakeDeploy.set(wantToIntake ? Value.kForward : Value.kReverse);

        double[] currentPowers = new double[5];
        if (wantToIntake || wantToShoot) {
            // Set to true once one of the sensors is empty
            boolean canGo = false;
            // loops from closest sensor to the farthest sensor
            for (int i = 4; i >= 0; i--) {
                if (wantToShoot && shooting && i == 4) {
                    currentPowers[i] = -0.75;
                    //in the next line, i >= X changes amount of balls able to intake(0=5balls)
                } else if (canGo || (i >= 0 && irSensors[i].get())) {
                    canGo = true;
                    currentPowers[i] = INTAKE_POWER[i];
                } else {
                    currentPowers[i] = 0;
                }
            }

        } else {
            for (int i = 0; i < 5; i++) {
                currentPowers[i] = 0;
            }
        }
        if (blinkyBackwards) {
            for (int i = 0; i < 5; i++) {
                currentPowers[i] = INTAKE_REVERSE_POWER[i];
            }
        }
        if (intakeBackwards) {
            currentPowers[0] = INTAKE_REVERSE_POWER[0];
        }

        intakeMotor.set(currentPowers[0]);
        for (int i = 0; i < 4; i++) {
            irMotors[i].set(ControlMode.PercentOutput, currentPowers[i + 1]);
        }
    }

    @Override
    public void teleopInit() {
        shooting = false;
    }

    @Override
    public void teleopPeriodic() {
        // deploy intake
        wantToIntake = controllers.leftTriggerHeld() >= 0.02;
        blinkyBackwards = controllers.joystickButton2();
        wantToShoot = controllers.joystickTriggerHeld();
        intakeBackwards = controllers.backButton();
    }

    public boolean getShooting() {
        return shooting;
    }

    public void setShooting(boolean shooting) {
        this.shooting = shooting;
    }

    public boolean ballReadyToShoot() {
        return !irSensors[4].get();
    }

    public boolean blinkyEmpty() {
        boolean activeSensor = false;
        for (DigitalInput irSensor : irSensors) {
            if (!irSensor.get()) {
                activeSensor = true;
                break;
            }
        }
        return !activeSensor;
    }


    public boolean blinkyFull(){
        boolean blinkyFull = false;
        if (!irSensors[4].get() && !irSensors[3].get() && !irSensors[2].get()){
            blinkyFull = true;
        }else{
            blinkyFull = false;
        }
        return blinkyFull;
    }

}
