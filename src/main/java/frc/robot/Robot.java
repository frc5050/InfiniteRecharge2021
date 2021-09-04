/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DeathStar;
import frc.robot.subsystems.DisturbingForce;
import frc.robot.autons.ShootAndMoveOffLine;
import frc.robot.autons.Slalom;
import frc.robot.autons.StraightLinePathWeaver;
import frc.robot.autons.TurnAuton;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PathLoader;
import frc.robot.subsystems.TacoTime;
import frc.robot.subsystems.Vader;
import frc.robot.tasks.IntakeComeBack;
import frc.robot.autons.AutonBase;
import frc.robot.autons.GalacticSearchA;
import frc.robot.autons.GalacticSearchB;
import frc.robot.autons.GalacticSearchFull;
import frc.robot.autons.IntakeUp;
import frc.robot.autons.MoveOffLineForward;
import frc.robot.autons.BarrelRacing;
import frc.robot.autons.BouncePath;
import frc.robot.autons.CirclePathWeaver;
import frc.robot.autons.Default;
import frc.robot.autons.DriveBackCalibration;
import frc.robot.autons.ShootAndDriveToTrench;
import frc.robot.autons.ShootAndGoToBrent;
import frc.robot.autons.VisionShootAndDriveToTrench;
import frc.robot.autons.VisionShootAndGoToBrent;
import frc.robot.autons.VisionShootAndMoveFar;
import frc.robot.autons.VisionShootAndMoveForward;
import frc.robot.autons.VisionShootAndMoveOffLine;
import frc.robot.autons.ShootAndMoveFar;
import frc.robot.autons.ShootAndMoveForward;
import frc.robot.subsystems.Blinky;

public class Robot extends TimedRobot {
    public static List<Subsystem> subsystems;
    public static List<AutonBase> autons;

    // Chooser
    private static SendableChooser<AutonBase> chooser;
    public AutonBase autonToRun;
    // Subsystems
    private DeathStar deathStar;
    private DriveTrain driveTrain;
    private Controllers controllers;
    private Vader vader;
    private Blinky blinky;
    private ColorSensor colorSensor;
    private TacoTime tacoTime;
    private Cameras cameras;
    private String autoSelected;
    private boolean joystickDpadPressed = false;
    private Limelight limelight;
    private PathLoader pathLoader;

    @Override
    public void robotInit() {
        //SmartDashboard.putNumber("FindPath/Left FeedForward", 0);
        //SmartDashboard.putNumber("FindPath/Right FeedForward", 0);

        //SmartDashboard.putNumber("FindPath/Left Power", 0);
        //SmartDashboard.putNumber("FindPath/Right Power", 0);

        chooser = new SendableChooser<>();
        subsystems = new ArrayList<>();
        autons = new ArrayList<>();
        cameras = new Cameras();
        // Subsystems
        controllers = Controllers.getInstance();
        driveTrain = DriveTrain.getInstance();
        deathStar = DeathStar.getInstance();
        vader = Vader.getInstance();
        blinky = Blinky.getInstance();
        colorSensor = ColorSensor.getInstance();
        tacoTime = TacoTime.getInstance();
        limelight = Limelight.getInstance();
        pathLoader = PathLoader.getInstance();
        // Autons
        new Default();
        new MoveOffLineForward();
        new DriveBackCalibration();
        new ShootAndMoveOffLine();
        new ShootAndDriveToTrench();
        new TurnAuton();
        new ShootAndGoToBrent();
        new ShootAndMoveForward();
        new ShootAndMoveFar();
        new VisionShootAndDriveToTrench();
        new VisionShootAndGoToBrent();
        new VisionShootAndMoveFar();
        new CirclePathWeaver();
        new VisionShootAndMoveForward();
        new VisionShootAndMoveOffLine();
        new IntakeUp();
        //Pathweaver
        new StraightLinePathWeaver();
        new Slalom();
        new BarrelRacing();
        new BouncePath();
        new GalacticSearchA();
        new GalacticSearchB();
        new GalacticSearchFull();

        SmartDashboard.putData("Selected Auton", chooser);
    }

    @Override
    public void teleopInit() {

        driveTrain.setMotorMode(IdleMode.kCoast);
        // blinky.generalInit();
        // autoToRun.done();

        vader.setVaderControlMode(new DisturbingForce(ControlMode.Position, vader.getVaderEncoder()));
        if (autonToRun != null) {
            autonToRun.done();
        }
        for (Subsystem subsystem : subsystems) {
            long startTime = System.nanoTime();
            subsystem.teleopInit();
            subsystem.generalInit();
            double timeTaken = System.nanoTime() - startTime;
            String name = subsystem.getClass().getName();
            //SmartDashboard.putNumber("Performance/TeleopInit/" + name, timeTaken / 1000000);
        }
    }



    @Override
    public void teleopPeriodic() {
        boolean zeroing = controllers.dPadLeft() || controllers.dPadRight() || controllers.dPadUp()
                || controllers.dPadDown();

        if (controllers.customPosition(true)) {
            double customDemand = SmartDashboard.getNumber("Vader/Position", 0);
            DisturbingForce disturbingForce = new DisturbingForce(ControlMode.Position, customDemand);
            vader.setVaderControlMode(disturbingForce);
        } else if (controllers.yellowZonePosition(true)) {
            vader.setVaderControlMode(controllers.isNew() ? Constants.NEW_YELLOW_ZONE_POSITION : Constants.CLOSE_POSITION);
        } else if (controllers.trenchPosition(true)) {
            vader.setVaderControlMode(Constants.TRENCH_POSITION);
        } else if (controllers.greenZonePosition(true)) {
            vader.setVaderControlMode(controllers.isNew() ? Constants.NEW_GREEN_ZONE_POSITION : Constants.VERY_CLOSE_POSITION);
        } else if (controllers.redZonePosition(true)) {
            vader.setVaderControlMode(controllers.isNew() ? Constants.NEW_RED_ZONE_POSITION : Constants.VERY_FAR_POSITION);
        } else if (controllers.blueZonePosition(true)) {
            vader.setVaderControlMode(controllers.isNew() ? Constants.NEW_BLUE_ZONE_POSITION : Constants.AUTOLINE_DISTURBING_FORCE);
        }
        if (controllers.joystickDPadUp()) {
            vader.setVaderControlMode(Constants.MANUAL_MODE_UP);
            joystickDpadPressed = true;
        } else if (controllers.joystickDPadDown()) {
            vader.setVaderControlMode(Constants.MANUAL_MODE_DOWN);
            joystickDpadPressed = true;
        } else if (joystickDpadPressed) {
            vader.setVaderControlMode(new DisturbingForce(ControlMode.Position, vader.getVaderEncoder()));
            joystickDpadPressed = false;
        }

        if (zeroing) {
            vader.setVaderControlMode(Constants.ZEROING);
        }
        for (Subsystem subsystem : subsystems) {
            long startTime = System.nanoTime();
            subsystem.teleopPeriodic();
            subsystem.generalPeriodic();
            //double timeTaken = System.nanoTime() - startTime;
            //String name = subsystem.getClass().getName();
           // SmartDashboard.putNumber("Performance/TeleopPeriodic/" + name, timeTaken / 1000000);
        }
        Limelight.getInstance().getDistance();
    }

    @Override
    public void autonomousInit() {

        autonToRun = chooser.getSelected();
        
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + autonToRun.getName());
        for (Subsystem subsystem : subsystems) {
            long startTime = System.nanoTime();
            subsystem.autonomousInit();
            subsystem.generalInit();
            double timeTaken = System.nanoTime() - startTime;
            String name = subsystem.getClass().getName();
            //SmartDashboard.putNumber("Performance/AutonomousInit/" + name, timeTaken / 1000000);


        }

        autonToRun.start();
    }

    @Override
    public void autonomousPeriodic() {
        if (autonToRun != null)
            autonToRun.periodic();
        for (Subsystem subsystem : subsystems) {
            long startTime = System.nanoTime();
            subsystem.autonomousPeriodic();
            subsystem.generalPeriodic();
            double timeTaken = System.nanoTime() - startTime;
            String name = subsystem.getClass().getName();
           // SmartDashboard.putNumber("Performance/AutonomousPeriodic/" + name, timeTaken / 1000000);
        }
    }

    public static void addAuton(AutonBase auton) {
        if (auton.isDefault()) {
            chooser.setDefaultOption(auton.getName(), auton);
        } else {
            chooser.addOption(auton.getName(), auton);
        }
    }
    @Override
    public void disabledInit(){
        driveTrain.setMotorMode(IdleMode.kCoast);
    }
}