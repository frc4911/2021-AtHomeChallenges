package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.WaitToFinishPathAction;
import frc.robot.auto.actions.BallSearchAction;
import frc.robot.auto.actions.CollectAction;
import frc.robot.auto.actions.PathSelection;


public class SearchMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Search Auto Mode Running!!!");
        // BallSearchAction ballSearchAction = new BallSearchAction();
        // runAction(ballSearchAction);
        // if (ballSearchAction.getPathSelection() == PathSelection.RED_A) {
        //     System.out.println("Galactic Search A Red Mode Running!!!");
        //     runAction(new CollectAction(true));
        //     runAction(new SetTrajectoryAction(trajectories.galacticSearchARedPath1.get(true), 180.0, 1.0));
        //     runAction(new WaitToFinishPathAction());
        //     runAction(new SetTrajectoryAction(trajectories.galacticSearchARedPath2.get(true), 180.0, 1.0));
        //     runAction(new WaitToFinishPathAction());
        //     runAction(new SetTrajectoryAction(trajectories.galacticSearchARedPath3.get(true), 180.0, 1.0));
        //     runAction(new WaitToFinishPathAction());
        //     runAction(new SetTrajectoryAction(trajectories.galacticSearchARedPath4.get(true), 180.0, 1.0));
        //     runAction(new WaitToFinishPathAction());
        //     runAction(new CollectAction(false));
        // } else if (ballSearchAction.getPathSelection() == PathSelection.RED_B) {
        //     System.out.println("Galactic Search B Red Mode Running!!!");
        //     runAction(new CollectAction(true));
        //     runAction(new SetTrajectoryAction(trajectories.galacticSearchBRedPath1.get(true), 0.0, 1.0));
        //     runAction(new WaitToFinishPathAction());
        //     runAction(new SetTrajectoryAction(trajectories.galacticSearchBRedPath2.get(true), 0.0, 1.0));
        //     runAction(new WaitToFinishPathAction());
        //     runAction(new SetTrajectoryAction(trajectories.galacticSearchBRedPath3.get(true), 0.0, 1.0));
        //     runAction(new WaitToFinishPathAction());
        //     runAction(new CollectAction(false));
        // } else if (ballSearchAction.getPathSelection() == PathSelection.BLUE_A) {
        //     System.out.println("Galactic Search A Blue Mode Running!!!");
        //     runAction(new CollectAction(true));
        //     runAction(new SetTrajectoryAction(trajectories.galacticSearchABluePath1.get(true), 0.0, 1.0));
        //     runAction(new WaitToFinishPathAction());
        //     runAction(new SetTrajectoryAction(trajectories.galacticSearchABluePath2.get(true), 0.0, 1.0));
        //     runAction(new WaitToFinishPathAction());
        //     runAction(new SetTrajectoryAction(trajectories.galacticSearchABluePath3.get(true), 0.0, 1.0));
        //     runAction(new WaitToFinishPathAction());
        //     runAction(new CollectAction(false));
        // } else if (ballSearchAction.getPathSelection() == PathSelection.BLUE_B) {
        //     System.out.println("Galactic Search B Blue Mode Running!!!");
        //     runAction(new CollectAction(true));
        //     runAction(new SetTrajectoryAction(trajectories.galacticSearchBBluePath1.get(true), 0.0, 1.0));
        //     runAction(new WaitToFinishPathAction());
        //     runAction(new SetTrajectoryAction(trajectories.galacticSearchBBluePath2.get(true), 0.0, 1.0));
        //     runAction(new WaitToFinishPathAction());
        //     runAction(new SetTrajectoryAction(trajectories.galacticSearchBBluePath3.get(true), 0.0, 1.0));
        //     runAction(new WaitToFinishPathAction());
        //     runAction(new CollectAction(false));
    
        // }

    }

}