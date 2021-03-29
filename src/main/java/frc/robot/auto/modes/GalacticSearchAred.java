package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.WaitToFinishPathAction;
import frc.robot.auto.actions.CollectAction;

public class GalacticSearchAred extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Galactic Search A Red Mode Running!!!");
        runAction(new CollectAction(true));
        runAction(new SetTrajectoryAction(trajectories.galacticSearchARedPath1.get(true), 180.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new SetTrajectoryAction(trajectories.galacticSearchARedPath2.get(true), 180.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new SetTrajectoryAction(trajectories.galacticSearchARedPath3.get(true), 180.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new SetTrajectoryAction(trajectories.galacticSearchARedPath4.get(true), 180.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new CollectAction(false));
    }

}