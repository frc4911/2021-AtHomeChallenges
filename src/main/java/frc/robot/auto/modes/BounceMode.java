package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.WaitToFinishPathAction;

public class BounceMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Bounce Auto Mode Running!!!");
        runAction(new SetTrajectoryAction(trajectories.bouncePathA.get(true), 0.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new SetTrajectoryAction(trajectories.bouncePathB.get(true), 0.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new SetTrajectoryAction(trajectories.bouncePathC.get(true), 0.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new SetTrajectoryAction(trajectories.bouncePathD.get(true), 0.0, 1.0));
        runAction(new WaitToFinishPathAction());
    }

}