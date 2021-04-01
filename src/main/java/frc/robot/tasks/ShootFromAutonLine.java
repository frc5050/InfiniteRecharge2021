/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tasks;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Blinky;
import frc.robot.subsystems.DeathStar;
import frc.robot.subsystems.Vader;

import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class ShootFromAutonLine implements TaskBase {
    private Timer timer;
    private Timer totalTimer;
    private boolean wasBlinkyEmpty;
    private final DeathStar deathStar = DeathStar.getInstance();
    private final Blinky blinky = Blinky.getInstance();

    @Override
    public void start() {
        deathStar.setOrder66(BLUE_ZONE_ORDER_66);
        Vader.getInstance().setVaderControlMode(BLUE_ZONE_POSITION);
        timer = new Timer();
        totalTimer = new Timer();
        totalTimer.start();
        blinky.wantToShoot = true;
    }

    @Override
    public boolean periodic() {
        if (blinky.blinkyEmpty() && !wasBlinkyEmpty) {
            timer.reset();
            timer.start();
            wasBlinkyEmpty = true;
        } else if (!blinky.blinkyEmpty() && wasBlinkyEmpty) {
            timer.stop();
            wasBlinkyEmpty = false;
        }
        return timer.get() >= 1 || totalTimer.get() >= 12;
    }

    @Override
    public void done() {
        timer.stop();
        deathStar.setOrder66(DONT_EXECUTE_ORDER_66);
        blinky.wantToShoot = false;

    }

}
