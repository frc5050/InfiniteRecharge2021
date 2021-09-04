/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Controllers;
import frc.robot.DriveSignal;
import frc.robot.Subsystem;


import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
    // Motors
    private CANSparkMax leftMotorFront;
    private CANSparkMax leftMotorBack;
    private CANSparkMax rightMotorFront;
    private CANSparkMax rightMotorBack;
    public CANEncoder leftEncoderFront;
    public CANEncoder leftEncoderBack;
    public CANEncoder rightEncoderFront;
    public CANEncoder rightEncoderBack;
    private CANPIDController leftMotorFrontPID;
    private CANPIDController leftMotorBackPID;
    private CANPIDController rightMotorFrontPID;
    private CANPIDController rightMotorBackPID;

    private DifferentialDriveOdometry odometry;
    public AHRS gyro;

    private double sumOfError = 0.0;
    private double change = 0.0;
    private Limelight limelight;
    private double angleToTarget;
    private int counterForVision;
    private static DriveTrain instance;
    private final Controllers controllers = Controllers.getInstance();
    

    private DriveTrain() {
        gyro = new AHRS(SPI.Port.kMXP);
        counterForVision = 0;
        leftMotorFront = new CANSparkMax(LEFT_DRIVE_MOTOR_FRONT, MotorType.kBrushless);
        leftMotorBack = new CANSparkMax(LEFT_DRIVE_MOTOR_BACK, MotorType.kBrushless);
        rightMotorFront = new CANSparkMax(RIGHT_DRIVE_MOTOR_FRONT, MotorType.kBrushless);
        rightMotorBack = new CANSparkMax(RIGHT_DRIVE_MOTOR_BACK, MotorType.kBrushless);
        leftEncoderFront = new CANEncoder(leftMotorFront);
        leftEncoderBack = new CANEncoder(leftMotorBack);
        rightEncoderFront = new CANEncoder(rightMotorFront);
        rightEncoderBack = new CANEncoder(rightMotorBack);
        leftMotorFrontPID = new CANPIDController(leftMotorFront);
        leftMotorBackPID = new CANPIDController(leftMotorBack);
        rightMotorFrontPID = new CANPIDController(rightMotorFront);
        rightMotorBackPID = new CANPIDController(rightMotorBack);
        leftMotorBack.enableVoltageCompensation(12);
        leftMotorFront.enableVoltageCompensation(12);
        rightMotorBack.enableVoltageCompensation(12);
        rightMotorFront.enableVoltageCompensation(12);
        leftMotorBack.follow(leftMotorFront);
        rightMotorBack.follow(rightMotorFront);
        resetEncoders();
        setPIDValue(0, 0, 0, 0, 0);
        angleToTarget = 0;


        limelight = Limelight.getInstance();
        limelight.setLED(true);
        odometry = new DifferentialDriveOdometry(getRotation2d());
        setPIDValue(PATHFINDER_P, PATHFINDER_I, PATHFINDER_D, PATHFINDER_FF, PATHFINDER_IZ);
    }

    public static DriveTrain getInstance(){
        if (instance == null){
            instance = new DriveTrain();
        }
        return instance;
    }

    public void setMotorMode(IdleMode mode) {
        leftMotorFront.setIdleMode(mode);
        leftMotorBack.setIdleMode(mode);
        rightMotorFront.setIdleMode(mode);
        rightMotorBack.setIdleMode(mode);
    }

    private void setPIDValue(double p, double i, double d, double ff, double iz) {

        leftMotorFrontPID.setP(p);
        leftMotorFrontPID.setI(i);
        leftMotorFrontPID.setD(d);
        leftMotorFrontPID.setFF(ff);
        leftMotorFrontPID.setIZone(iz);

        /*leftMotorBackPID.setP(p);
        leftMotorBackPID.setI(i);
        leftMotorBackPID.setD(d);
        leftMotorBackPID.setFF(ff);
        leftMotorBackPID.setIZone(iz); */

        rightMotorFrontPID.setP(p);
        rightMotorFrontPID.setI(i);
        rightMotorFrontPID.setD(d);
        rightMotorFrontPID.setFF(ff);
        rightMotorFrontPID.setIZone(iz);

        /*rightMotorBackPID.setP(p);
        rightMotorBackPID.setI(i);
        rightMotorBackPID.setD(d);
        rightMotorBackPID.setFF(ff);
        rightMotorBackPID.setIZone(iz); */
    }

    public double getAngle() {
        return gyro.getAngle();
    }

    public Rotation2d getRotation2d() {
        // Returns the gyroscope as a Rotation2d (stores sine and cosine of angle)
        return new Rotation2d(-getAngle() * (Math.PI / 180.0));
    }

    public void resetGyro() {
        gyro.reset();
    }

    @Override
    public void generalInit() {
        resetGyro();
        rightMotorFront.setInverted(false);
        rightMotorBack.setInverted(false);
        leftMotorBack.setInverted(true);
        leftMotorFront.setInverted(true);
        setMotorPower(0, 0);

        SmartDashboard.putNumber("DriveTrain/Vision Loop P", 0.8);
        SmartDashboard.putNumber("DriveTrain/Vision Loop d", 0.0);
        SmartDashboard.putNumber("DriveTrain/Vision Loop i", 0.0001);

       // SmartDashboard.putNumber("DriveTrain P Values", 0);
        //SmartDashboard.putNumber("DriveTrain I Values", 0);
        //SmartDashboard.putNumber("DriveTrain D Values", 0);
        //SmartDashboard.putNumber("DriveTrain FF Values", 0);
        //SmartDashboard.putNumber("DriveTrain Iz Values", 0);
    }

    @Override
    public void autonomousInit(){
        setMotorMode(IdleMode.kBrake);
    }

    double averageLeft = 0;
    double averageRight = 0;

    /*
     * Power,LeftAvg,RightAvg 0.05,183.077,168.668 0.1,451.458,433.115
     * 0.2,970.786,909.829 0.3,1460.02,1386.026
     */
    @Override
    public void generalPeriodic() {
        //System.out.println(gyro.getAngle());
        SmartDashboard.putNumber("DriveTrain/Left Front Encoder Value", leftEncoderFront.getPosition());
        SmartDashboard.putNumber("DriveTrain/Left Back Encoder Value", leftEncoderBack.getPosition());
        SmartDashboard.putNumber("DriveTrain/Right Front Encoder Value", rightEncoderFront.getPosition());
        SmartDashboard.putNumber("DriveTrain/Right Back Encoder Value", rightEncoderBack.getPosition());
        SmartDashboard.putNumber("DriveTrain/Left Front Velocity Value", leftEncoderFront.getVelocity());
        SmartDashboard.putNumber("DriveTrain/Left Back Velocity Value", leftEncoderBack.getVelocity());
        SmartDashboard.putNumber("DriveTrain/Right Front Velocity Value", rightEncoderFront.getVelocity());
        SmartDashboard.putNumber("DriveTrain/Right Back Velocity Value", rightEncoderBack.getVelocity()); 
        SmartDashboard.putNumber("DriveTrain/Measurement Period", rightEncoderFront.getMeasurementPeriod());
        SmartDashboard.putNumber("DriveTrain/Average Depth", rightEncoderFront.getAverageDepth());

        odometry.update(
            getRotation2d(), 
            -leftEncoderFront.getPosition() * (DISTANCE_PER_REVOLUTION_METERS), 
            -rightEncoderFront.getPosition() * (DISTANCE_PER_REVOLUTION_METERS)
        );
    }

    @Override
    public void teleopPeriodic() {

        DriveSignal signal = controllers.curvatureDrive();

       /* if (controllers.useVisionPressed()) {
            limelight.setLED(true);
        } else if (controllers.useVisionReleased()) {
            limelight.setLED(false);
        }*/
        if (controllers.useVision()) {
            visionLoop();
        } else {
            setMotorPowerSignal(signal);
        }
    }

    public void setMotorPowerSignal(DriveSignal signal) {
        setMotorPower(signal.getLeftPower(), signal.getRightPower(), signal.getControlType());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            // Converts each unit from RPM to mm/m to mm/s to m/s
            (leftEncoderFront.getVelocity() * DISTANCE_PER_REVOLUTION_METERS) / 60.0,
            (rightEncoderFront.getVelocity() * DISTANCE_PER_REVOLUTION_METERS) / 60.0
        );
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();

        odometry.resetPosition(pose, getRotation2d());
    }

    public void setMotorPower(double leftPower, double rightPower) {
        /*System.out.print(leftPower);
        System.out.print(", ");
        System.out.println(rightPower);*/
        setMotorPower(leftPower, rightPower, ControlType.kDutyCycle);
    }

    public void setMotorPower(double leftPower, double rightPower, ControlType controlType) {
        /*System.out.print(leftPower);
        System.out.print(", ");
        System.out.println(rightPower);*/
        leftMotorFrontPID.setReference(leftPower, controlType);
        rightMotorFrontPID.setReference(rightPower, controlType);
    }

    public void resetEncoders() {
        leftEncoderFront.setPosition(0);
        leftEncoderBack.setPosition(0);
        rightEncoderFront.setPosition(0);
        rightEncoderBack.setPosition(0);
    }

    public void setDemand(DriveSignal signal, ControlType mode) {
        leftMotorFrontPID.setReference(signal.getLeftPower(), mode);
        leftMotorBackPID.setReference(signal.getLeftPower(), mode);
        rightMotorFrontPID.setReference(signal.getRightPower(), mode);
        rightMotorBackPID.setReference(signal.getRightPower(), mode);
    }

    public void visionLoop() {
        if (limelight.seesTarget()) {
            double previousAngleToTarget = angleToTarget;
            double currentAngle = limelight.getAngleToTarget() - 1;
            double delta = (currentAngle - angleToTarget);
            angleToTarget = currentAngle;
            change = Math.abs(previousAngleToTarget - angleToTarget);
            sumOfError = sumOfError + currentAngle;
            double p = SmartDashboard.getNumber("DriveTrain/Vision Loop P", 0.85) / 90.0;
            double d = SmartDashboard.getNumber("DriveTrain/Vision Loop d", 0.0) / 90.0;
            double i = SmartDashboard.getNumber("DriveTrain/Vision Loop i", 0.0001) / 90.0;
            double leftPower = -angleToTarget * p + d * delta + i * sumOfError;
            double rightPower = angleToTarget * p + d * delta + i * sumOfError;
            DriveSignal driveSignal = new DriveSignal(leftPower, rightPower);
            setMotorPowerSignal(driveSignal);
            SmartDashboard.putNumber("vision loop/ right motor power", rightPower);
            SmartDashboard.putNumber("vision loop/ angle to target", angleToTarget);

        } else {
            setMotorPowerSignal(new DriveSignal(0, 0));
        }
    }

    public boolean isAimedAtTarget() {
        if ((Math.abs(angleToTarget) <= 3) && change < 0.01) {
            counterForVision++;
        } else {
            counterForVision = 0;
        }
        return counterForVision >= 3;
    }


    public void straightLineLoop(double desiredAngle, double power) {
        double currentAngle = gyro.getAngle();
        double error = desiredAngle - currentAngle;
        double p = 1.0 / 90.0;
        double leftPower = -error * p + power;
        double rightPower = error * p + power;
        DriveSignal driveSignal = new DriveSignal(leftPower, rightPower * 1.03);
        setMotorPowerSignal(driveSignal);
    }

}
