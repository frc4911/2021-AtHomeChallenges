package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.BallSearchAction;
import frc.robot.auto.actions.SetTrajectoryAction;

public class TestMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        BallSearchAction ballSearchAction = new BallSearchAction();
        runAction(ballSearchAction);
        if (ballSearchAction.getPathSelection() == PathSelection.RED_A) {
            runAction(new SetTrajectoryAction(trajectories.redAPath.get(true), 0.0, 1.0)); 
        } else if (ballSearchAction.getPathSelection() == PathSelection.RED_B) {
            runAction(new SetTrajectoryAction(trajectories.redBPath.get(true), 0.0, 1.0));
        } else if (ballSearchAction.getPathSelection() == PathSelection.BLUE_A) {
            runAction(new SetTrajectoryAction(trajectories.blueAPath.get(true), 0.0, 1.0));
        } else if (ballSearchAction.getPathSelection() == PathSelection.BLUE_B) {
            runAction(new SetTrajectoryAction(trajectories.blueBPath.get(true), 0.0, 1.0));
        }

        // runAction(new CollectAction());
        // runAction(new SetTrajectoryAction(trajectories.testPath3.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // runAction(new SetTrajectoryAction(trajectories.testPath4.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // runAction(new ShootAction(5.0));
        // runAction(new ShootAction(5.0));
        // runAction(new WaitAction(10.0));
        // runAction(new SetTrajectoryAction(trajectories.testPath2.get(true), 0.0, 1.0));
        // runAction(new SetTrajectoryAction(trajectories.testPathBrian.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // runAction(new CollectAction());
    }

}