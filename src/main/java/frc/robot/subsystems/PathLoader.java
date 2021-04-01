/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Subsystem;

import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class PathLoader extends Subsystem {
    private static PathLoader instance;
    private HashMap<String, Trajectory> hashPaths = new HashMap<>();

    private PathLoader() throws FileNotFoundException {
        File pathFile = Filesystem.getDeployDirectory().toPath().resolve("paths/output").toFile();
        if (!pathFile.exists()) {
            throw new FileNotFoundException(pathFile.toString());
        }
        File[] paths = pathFile.listFiles();

        /*
        var voltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                PATHFINDER_VOLTS,
                PATHFINDER_VOLT_SECONDS_PER_METER,
                PATHFINDER_VOLT_SECONDS_SQUARED_PER_METER
            ),
            PATHFINDER_DRIVE_KINEMATICS,
            PATHFINDER_MAX_VOLTS
        );

        TrajectoryConfig config = new TrajectoryConfig(PATHFINDER_MAX_SPEED, PATHFINDER_MAX_ACCELERATION)
            .setKinematics(PATHFINDER_DRIVE_KINEMATICS)
            .addConstraint(voltageConstraint);
        */

        System.out.println("Loading Paths");
        for (File path : paths) {
            try {
                Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(path.toPath());
                //System.out.println(path.getName());
                hashPaths.put(path.getName(), trajectory);
            } catch (IOException e) {
                DriverStation.reportError("Failed at " + path.getName(), e.getStackTrace());
            }
       }
   }

   public Trajectory getPath(String pathName) throws FileNotFoundException {
       Trajectory trajectory = hashPaths.get(pathName);
       System.out.println(pathName + "!!!!");
       if (trajectory == null) {
           throw new FileNotFoundException(pathName);
       }

       return trajectory;
   }

   public static PathLoader getInstance(){
    if (instance == null){
            try {
                instance = new PathLoader();
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
    }
    return instance;
}


}
