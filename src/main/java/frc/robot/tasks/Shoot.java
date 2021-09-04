/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tasks;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Order66;
import frc.robot.subsystems.Blinky;
import frc.robot.subsystems.DeathStar;
import frc.robot.subsystems.DisturbingForce;
import frc.robot.subsystems.Vader;

import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class Shoot implements TaskBase {
    private Timer timer;
    private boolean wasBlinkyEmpty;
    private Order66 order66;
    private DisturbingForce disturbingForce;
    private final Blinky blinky = Blinky.getInstance();
    private final DeathStar deathStar = DeathStar.getInstance();

    public Shoot(Order66 order66, DisturbingForce disturbingForce) {
        this.order66 = order66;
        this.disturbingForce = disturbingForce;
    }

    @Override
    public void start() {
        System.out.println("shooting started");
        timer = new Timer();
        blinky.wantToShoot = true;
        deathStar.setOrder66(order66);
        Vader.getInstance().setVaderControlMode(disturbingForce);
        
    }

    @Override
    public boolean periodic() {
        if (blinky.blinkyEmpty() && !wasBlinkyEmpty) {
            // newly empty
            timer.reset();
            timer.start();
            wasBlinkyEmpty = true;
        } else if (!blinky.blinkyEmpty() && wasBlinkyEmpty) {
            timer.stop();
            wasBlinkyEmpty = false;
        }

        if (blinky.blinkyEmpty()) {
            return timer.get() >= 1.0;
        }
        // if blinky is empty & it's been at least 1 second
        // or if blinky isn't empty then if it's been at least 5 seconds
        return timer.get() >= 5.0;
        
    }

    @Override
    public void done() {
        timer.stop();
        deathStar.setOrder66(DONT_EXECUTE_ORDER_66);
        blinky.wantToShoot = false;
        System.out.println("Shooting ended");

    }
}
