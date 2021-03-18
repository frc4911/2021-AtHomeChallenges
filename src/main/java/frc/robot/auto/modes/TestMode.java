package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.LLTest;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.WaitToFinishPathAction;

public class TestMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        // runAction(new CollectAction());
        // runAction(new SetTrajectoryAction(trajectories.testPath3.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // runAction(new SetTrajectoryAction(trajectories.testPath4.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // runAction(new ShootAction(5.0));
        // runAction(new LLTest());
        // runAction(new ShootAction(5.0));
        // runAction(new WaitAction(10.0));
        runAction(new SetTrajectoryAction(trajectories.testPath.get(true), 0.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new SetTrajectoryAction(trajectories.testPath2.get(true), 0.0, 1.0));
        runAction(new WaitToFinishPathAction());
        runAction(new SetTrajectoryAction(trajectories.testPath3.get(true), 0.0, 1.0));
        runAction(new WaitToFinishPathAction());
        // runAction(new SetTrajectoryAction(trajectories.testPath4.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // runAction(new CollectAction());
    }

}