package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.WaitToFinishPathAction;

public class SlalomMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Slalom Auto Mode Running!!!");
        runAction(new SetTrajectoryAction(trajectories.slalomPath.get(true), 0.0, 1.0));
        runAction(new WaitToFinishPathAction());
    }

}