/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tasks;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.DriveSignal;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.*;
import com.revrobotics.ControlType;

/**
 * Add your docs here.
 */
public class FindPath implements TaskBase {

    Trajectory trajectory;
    RamseteController controller;
    SimpleMotorFeedforward feedforward;
    Timer timer;

  


    public FindPath(Trajectory trajectory) {
        System.out.println("find path call");
        this.trajectory = trajectory;
        
        
    }

    @Override
    public void start() {
        System.out.println("task starting");
       //Try to put print for path names here
    
        feedforward = new SimpleMotorFeedforward(
            PATHFINDER_VOLTS,
            PATHFINDER_VOLT_SECONDS_PER_METER,
            PATHFINDER_VOLT_SECONDS_SQUARED_PER_METER
        );

        DriveTrain.getInstance().resetOdometry(trajectory.getInitialPose());
        controller = new RamseteController(RAMSETE_B, RAMSETE_ZETA);
        timer = new Timer();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean periodic() {
        double currentTime = timer.get();

        Trajectory.State state = trajectory.sample(Math.min(currentTime, trajectory.getTotalTimeSeconds()));
        // thids line
        Pose2d pose = DriveTrain.getInstance().getPose();
        ChassisSpeeds speeds = controller.calculate(pose, state);
        var wheelSpeeds = PATHFINDER_DRIVE_KINEMATICS.toWheelSpeeds(speeds);
        DriveSignal signal = new DriveSignal(
            -wheelSpeeds.leftMetersPerSecond, //* 60.0 / DISTANCE_PER_REVOLUTION_METERS, 
            -wheelSpeeds.rightMetersPerSecond, //* 60.0 / DISTANCE_PER_REVOLUTION_METERS,
            ControlType.kVoltage
        );

        /*SmartDashboard.putNumber("X-value", pose.getTranslation().getX());
        SmartDashboard.putNumber("Y-value", pose.getTranslation().getY());

        SmartDashboard.putNumber("FindPath/Left Motor Speed",  signal.getLeftPower());
        SmartDashboard.putNumber("FindPath/Right Motor Speed",  signal.getRightPower());
        SmartDashboard.putNumber("FindPath/Gyro Angle",  DriveTrain.getInstance().getAngle());*/

        DriveTrain.getInstance().setMotorPowerSignal(signal);
      
        /*SmartDashboard.putNumber("Trajectory Time", trajectory.getTotalTimeSeconds());
        SmartDashboard.putNumber("Timer", timer.get());
        SmartDashboard.putBoolean("Trajectory Done",  timer.hasElapsed(trajectory.getTotalTimeSeconds()));*/

        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void done() {
        timer.stop();
    }
}
