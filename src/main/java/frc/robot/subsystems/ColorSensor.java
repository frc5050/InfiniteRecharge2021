/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Controllers;
import frc.robot.Subsystem;

import com.revrobotics.ColorMatchResult;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class ColorSensor extends Subsystem {
    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher;
    private int changes;
    private char gameData;
    private final Servo colorServo;
    private char lastFieldColor;
    private Timer timer;
    public boolean colorSpinManualMode;
    private static ColorSensor instance;
    private final Controllers controllers;

    // Spin Color Wheel
    Solenoid colorWheelSolenoid;
    CANSparkMax colorWheelMotor;
    CANEncoder colorWheelEncoder;

    private ColorSensor() {
        changes = 0;
        // Wheel Turn
        colorWheelSolenoid = new Solenoid(PCM, COLOR_SOLENOID);
        colorWheelMotor = new CANSparkMax(COLOR_WHEEL_BALANCE_MOTOR, MotorType.kBrushless);
        colorWheelEncoder = colorWheelMotor.getEncoder();
        // Color Position Turn
        colorSensor = new ColorSensorV3(COLOR_SENSOR_I2C_PORT);
        colorMatcher = new ColorMatch();

        colorMatcher.addColorMatch(SQUIRTLE_TARGET_COLOR);
        colorMatcher.addColorMatch(CHARMANDER_TARGET_COLOR);
        colorMatcher.addColorMatch(BULBASAUR_TARGET_COLOR);
        colorMatcher.addColorMatch(PIKACHU_TARGET_COLOR);
        colorServo = new Servo(COLOR_SERVO);

        timer = new Timer();

        controllers = Controllers.getInstance();
    }

    public static ColorSensor getInstance(){
      if (instance == null){
        instance = new ColorSensor();
      }
      return instance;
    }

    @Override
    public void generalInit() {
        colorSpinManualMode = false;
        changes = 0;
        colorWheelSolenoid.set(false);
        colorServo.set(COLOR_SERVO_DOWN);
        timer.stop();
        timer.reset();
        colorWheelMotor.set(0);
    }

    public char getColor() {

        Color detectedColor = colorSensor.getColor();
        double IR = colorSensor.getIR();
       // SmartDashboard.putNumber("Color Sensor/Red", detectedColor.red);
        //SmartDashboard.putNumber("Color Sensor/Green", detectedColor.green);
        //SmartDashboard.putNumber("Color Sensor/Blue", detectedColor.blue);
        //SmartDashboard.putNumber("IR", IR);
        String colorString;
        char fieldColor = 'U';
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        // colorString is what our color sensor is seeing, fieldColor is what the field
        // should be seeing if we are seeing a certain color
        if (match.color == SQUIRTLE_TARGET_COLOR) {
            colorString = "Blue";
            fieldColor = 'R';
        } else if (match.color == CHARMANDER_TARGET_COLOR) {
            colorString = "Red";
            fieldColor = 'B';

        } else if (match.color == BULBASAUR_TARGET_COLOR) {
            colorString = "Green";
            fieldColor = 'Y';
        } else if (match.color == PIKACHU_TARGET_COLOR) {
            colorString = "Yellow";
            fieldColor = 'G';
        } else {
            colorString = "Unknown";
        }
        //SmartDashboard.putNumber("Color Sensor/Confidence", match.confidence);
        //SmartDashboard.putString("Color Sensor/Detected Color", colorString);
        return fieldColor;
    }

    @Override
    public void teleopPeriodic() {

        // Wheel Turn
        if (controllers.aHeld()) {
            colorServo.set(COLOR_SERVO_UP);
            colorWheelSolenoid.set(true);
        } else if (colorServo.get() != COLOR_SERVO_DOWN) {
            colorServo.set(COLOR_SERVO_DOWN);
        } else if (colorServo.get() == COLOR_SERVO_DOWN) {
            colorWheelSolenoid.set(false);
        }

      //  SmartDashboard.putNumber("Color Sensor/Changes", changes);

        if (controllers.bPressed()) {
            String gameSpecificMessage = DriverStation.getInstance().getGameSpecificMessage();
            if (gameSpecificMessage.length() > 0) {
                gameData = gameSpecificMessage.charAt(0);
            } else {
                gameData = 'U';
            }
            colorSpinManualMode = false;
        }
        if (controllers.yPressed()) {
            colorSpinManualMode = false;
            changes = 0;
        }
        if (controllers.yHeld() && changes < 32) {
            colorWheelMotor.set(1.0);
            char fieldColor = getColor();
            if (timer.hasPeriodPassed(0.1)) {
                changes += 1;
                timer.stop();
                timer.reset();
            }
            if (lastFieldColor != fieldColor) {
                timer.reset();
                timer.start();
                lastFieldColor = fieldColor;
            }
        }
        // this sees if the fieldColor(based off of our colorString) is equal to the
        // color FMS is giving usgameData
        else if (controllers.bHeld() && getColor() != gameData && gameData != 'U') {
            colorWheelMotor.set(-0.5);
        } else if (!colorSpinManualMode) {
            colorWheelMotor.set(0);
        }
        // if (Robot.controllers.getGamepadX(Hand.kRight) >= 0.02){
        // colorSpinManualMode = true;
        // colorWheelMotor.set(Robot.controllers.getGamepadX(Hand.kRight));
        // }
    }
}
