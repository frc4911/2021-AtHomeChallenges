package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.WaitToFinishPathAction;
import frc.robot.auto.actions.CollectAction;

public class GalacticSearchBblue extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Galactic Search B Blue Mode Running!!!");
        // runAction(new CollectAction(true));
        // runAction(new SetTrajectoryAction(trajectories.galacticSearchBBluePath1.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // runAction(new SetTrajectoryAction(trajectories.galacticSearchBBluePath2.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // runAction(new SetTrajectoryAction(trajectories.galacticSearchBBluePath3.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // runAction(new CollectAction(false));
    }

}