/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Constants.*;

import com.revrobotics.ControlType;

/**
 * Add your docs here.
 */
public class Controllers extends Subsystem {
  private Joystick joystick;
  private Joystick secondaryJoystick;
  private XboxController gamepad;
  public boolean autoLineShoot = false;
  private static Controllers instance;
  private double accumulator;

  private boolean newButtons = false;

  private Controllers() {
    joystick = new Joystick(DRIVER_JOYSTICK);
    secondaryJoystick = new Joystick(SECONDARY_JOYSTICK);
    gamepad = new XboxController(OPERATOR_GAMEPAD);
  }

  public static final Controllers getInstance() {
    if (instance == null) {
      instance = new Controllers();
    }
    return instance;
  }

  @Override
  public void teleopInit() {
    newButtons = false;
    SmartDashboard.putBoolean("Controllers/New Buttons", newButtons);
  }

  @Override
  public void teleopPeriodic() {
    if (joystick.getRawButtonPressed(12)) {
      newButtons = !newButtons;
      SmartDashboard.putBoolean("Controllers/New Buttons", newButtons);
    }
  }

  /**
   * @param value
   * @param deadband
   * @return
   */
  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public DriveSignal arcadeDrive() {
    double yInput = joystick.getY();
    double xInput = joystick.getX();

    yInput = MathUtil.clamp(yInput, -1.0, 1.0);
    yInput = applyDeadband(yInput, 0.09);

    xInput = -MathUtil.clamp(xInput, -1.0, 1.0);
    xInput = applyDeadband(xInput, 0.09);

    if (yellowZonePosition(true) || blueZonePosition(true) || trenchPosition(true)) {
      yInput /= 2;
      xInput /= 2;
    }
    double leftPower;
    double rightPower;

    double maxInput = Math.copySign(Math.max(Math.abs(yInput), Math.abs(xInput)), yInput);

    if (yInput >= 0.0) {
      if (xInput >= 0.0) {
        leftPower = maxInput;
        rightPower = yInput - xInput;
      } else {
        leftPower = yInput + xInput;
        rightPower = maxInput;
      }
    } else {
      if (xInput >= 0.0) {
        leftPower = yInput + xInput;
        rightPower = maxInput;
      } else {
        leftPower = maxInput;
        rightPower = yInput - xInput;
      }
    }

    leftPower = MathUtil.clamp(leftPower, -1.0, 1.0);
    rightPower = MathUtil.clamp(rightPower, -1.0, 1.0);
    DriveSignal driveSignal = new DriveSignal(leftPower, rightPower);
    // System.out.println(leftPower);
    // System.out.println(rightPower);
    return driveSignal;
  }
  public DriveSignal curvatureDrive() {

    double xSpeed = joystick.getY();
    double zRotation = -secondaryJoystick.getX();
    
    boolean isQuickTurn = secondaryJoystick.getTrigger();

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    xSpeed = applyDeadband(xSpeed, 0.09);

    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    zRotation = applyDeadband(zRotation, 0.09);

    double angularPower;
    boolean overPower;

    if (isQuickTurn) {
      if (Math.abs(xSpeed) < QUICK_STOP_THRESHOLD) {
        accumulator =
            (1 - QUICK_STOP_ALPHA) * accumulator
                + QUICK_STOP_ALPHA * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
      }
      overPower = true;
      angularPower = zRotation / 4;
    } else {
      overPower = false;
      angularPower = Math.abs(xSpeed) * zRotation - accumulator;

      if (accumulator > 1) {
        accumulator -= 1;
      } else if (accumulator < -1) {
        accumulator += 1;
      } else {
        accumulator = 0.0;
      }
    }

    double leftMotorOutput = xSpeed + angularPower;
    double rightMotorOutput = xSpeed - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }

    return new DriveSignal(leftMotorOutput, rightMotorOutput);

  }
  public boolean joystickTriggerHeld() {
    return joystick.getRawButton(JOYSTICK_TRIGGER) || autoLineShoot;
  }

  public boolean yHeld() {
    return gamepad.getYButton();
  }

  public boolean yPressed() {
    return gamepad.getYButtonPressed();
  }

  public boolean bHeld() {
    return gamepad.getBButton();
  }

  public boolean bPressed() {
    return gamepad.getBButtonPressed();
  }

  public boolean aHeld() {
    return gamepad.getAButton();
  }

  public boolean xHeld() {
    return gamepad.getXButton();
  }

  public double getGamepadY(Hand hand) {
    return gamepad.getY(hand);
  }

  public double getGamepadX(Hand hand) {
    return gamepad.getX(hand);
  }

  public boolean leftBumperHeld() {
    return gamepad.getBumper(Hand.kLeft);
  }

  public boolean rightBumperHeld() {
    return gamepad.getBumper(Hand.kRight);
  }

  public double rightTriggerHeld() {
    return gamepad.getTriggerAxis(Hand.kRight);
  }

  public double leftTriggerHeld() {
    return gamepad.getTriggerAxis(Hand.kLeft);
  }

  public boolean dPadUp() {
    int pov = gamepad.getPOV();
    return pov == 0 || pov == 30 || pov == 330;
  }

  public boolean dPadDown() {
    int pov = gamepad.getPOV();
    return pov == 180 || pov == 150 || pov == 210;
  }

  public boolean dPadRight() {
    int pov = gamepad.getPOV();
    return pov == 90 || pov == 110 || pov == 60;
  }

  public boolean dPadLeft() {
    int pov = gamepad.getPOV();
    return pov == 270 || pov == 300 || pov == 240;
  }

  public boolean yellowZonePosition(boolean getPressed) {
    return getPressed ? secondaryJoystick.getRawButtonPressed(4) : secondaryJoystick.getRawButton(4);
    //close
    //7 and then 9
  }

  public boolean blueZonePosition(boolean getPressed) {
    return getPressed ? secondaryJoystick.getRawButtonPressed(5) : secondaryJoystick.getRawButton(5);
    //autoLine
    // 9 and then 5
  }

  public boolean greenZonePosition(boolean getPressed) {
    return getPressed ? secondaryJoystick.getRawButtonPressed(6) : secondaryJoystick.getRawButton(6);
    //veryClose
    // 8
  }

  public boolean redZonePosition(boolean getPressed) {
    return getPressed ? secondaryJoystick.getRawButtonPressed(3) : secondaryJoystick.getRawButton(3);
    //veryFar
    // 12 and then 8
  }

  public boolean spinWithoutMoveHood(boolean getPressed){
    return getPressed ? joystick.getRawButtonPressed(3) : joystick.getRawButton(3);
  }

  public boolean trenchPosition(boolean getPressed) {
    return false;
    //return getPressed ? joystick.getRawButtonPressed(11) : joystick.getRawButton(11);
    //trench
    //11
  }

  public boolean customPosition(boolean getPressed) {
    return getPressed ? joystick.getRawButtonPressed(11) : joystick.getRawButton(11);
  }

  public boolean joystickDPadUp() {
    int pov = joystick.getPOV();
    return pov == 0 || pov == 45 || pov == 315;
  }

  public boolean joystickDPadDown() {
    int pov = joystick.getPOV();
    return pov == 180 || pov == 225 || pov == 135;
  }

  public boolean joystickDPadLeft() {
    int pov = joystick.getPOV();
    return pov == 270;
  }

  public boolean joystickDPadRight() {
    int pov = joystick.getPOV();
    return pov == 90;
  }

  public boolean backButton() {
    return gamepad.getBackButton();
  }

  public boolean joystickButton2() {
    return joystick.getRawButton(2);
  }

  public boolean isNew() {
    return newButtons;
  }

  public Order66 getOrder66(Order66 oldValue) {
    Order66 newValue;
    if (customPosition(false)) {
      double customDemand = SmartDashboard.getNumber("Shooter/Shooter Speed", 0);
      newValue = new Order66(ControlType.kVelocity, customDemand);
    } else if (greenZonePosition(false)) {
      newValue = isNew() ? NEW_GREEN_ZONE_ORDER_66 : VERY_CLOSE_ORDER_66;
    } else if (yellowZonePosition(false)) {
      newValue = isNew() ? NEW_YELLOW_ZONE_ORDER_66 : CLOSE_ORDER_66;
    } else if (blueZonePosition(false)) {
      newValue = isNew() ? NEW_BLUE_ZONE_ORDER_66 : AUTOLINE_ORDER_66;
    } else if (trenchPosition(false)) {
      newValue = TRENCH_ORDER_66;
    } else if (redZonePosition(false)) {
      newValue = isNew() ? NEW_RED_ZONE_ORDER_66 : VERY_FAR_ORDER_66;
    } else if (spinWithoutMoveHood(false)){
      newValue = isNew() ? NEW_YELLOW_ZONE_ORDER_66 : CLOSE_ORDER_66;
    }else {
      
      newValue = DONT_EXECUTE_ORDER_66;
    }

    if (newValue.demand != oldValue.demand) {
      SmartDashboard.putNumber("Shooter/Shooter Speed", newValue.demand);
    }
    return newValue;
  }

  public boolean joystickButton3Pressed() {
    return joystick.getRawButtonPressed(3);
  }

  public boolean joystickButton4Pressed() {
    return joystick.getRawButtonPressed(4);
  }

  public boolean joystickButton6Pressesd() {
    return joystick.getRawButtonPressed(6);
  }

  public boolean useVisionPressed() {
    return joystick.getRawButtonPressed(4);
  }

  public boolean useVisionReleased() {
    return joystick.getRawButtonReleased(4);
  }

  public boolean useVision() {
    return joystick.getRawButton(4);
    //10
  }

}
