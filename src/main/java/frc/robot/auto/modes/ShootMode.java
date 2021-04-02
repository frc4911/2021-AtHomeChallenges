package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.WaitToFinishPathAction;
import frc.robot.auto.actions.ManualShootAction;

public class ShootMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Shoot Auto Mode Running!!!");
        // runAction(new SetTrajectoryAction(trajectories.barrelPath.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // runAction(new WaitAction(2.0));
        // runAction(new ShootAction(3.0));
        runAction(new ManualShootAction(5.0,3700.0,3));
        // runAction(new SetTrajectoryAction(trajectories.backAwayFromLinePath.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
    }
}