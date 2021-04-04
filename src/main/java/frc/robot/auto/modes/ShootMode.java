package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.WaitToFinishPathAction;
import frc.robot.auto.actions.ManualShootAction;
import frc.robot.auto.actions.CollectAction;
import frc.robot.auto.actions.WaitAction; 

public class ShootMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Shoot Auto Mode Running!!!");
        //while (true) {
            runAction(new ManualShootAction(5.0,3700.0,3));
            runAction(new SetTrajectoryAction(trajectories.powerPortBackwardPath.get(true), 0.0, 1.0));
            runAction(new WaitToFinishPathAction());
            runAction(new CollectAction(true));
            runAction(new WaitAction(5.0));
            runAction(new CollectAction(false));
            runAction(new SetTrajectoryAction(trajectories.powerPortForwardPath.get(true), 0.0, 1.0));
            runAction(new WaitToFinishPathAction());
        //}/
        // runAction(new SetTrajectoryAction(trajectories.backAwayFromLinePath.get(true), 0.0, 1.0));   
    }
}