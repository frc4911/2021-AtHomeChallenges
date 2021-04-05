package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.WaitToFinishPathAction;
import frc.robot.auto.actions.CollectAction;

public class GalacticSearchAblue extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Galactic Search A Blue Mode Running!!!");
        // runAction(new CollectAction(true));
        // runAction(new SetTrajectoryAction(trajectories.galacticSearchABluePath1.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // runAction(new SetTrajectoryAction(trajectories.galacticSearchABluePath2.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // runAction(new SetTrajectoryAction(trajectories.galacticSearchABluePath3.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // runAction(new CollectAction(false));
    }

}