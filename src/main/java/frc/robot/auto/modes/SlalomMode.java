package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.WaitToFinishPathAction;

public class SlalomMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Slalom Auto Mode Running!!!");
        //1
        runAction(new SetTrajectoryAction(trajectories.slalomPath.get(true), 0.0, 1.0));
        runAction(new WaitToFinishPathAction());
        // //2
        // runAction(new SetTrajectoryAction(trajectories.slalomPathStraightF.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // //3
        // runAction(new SetTrajectoryAction(trajectories.slalomPathRound.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // //4
        // runAction(new SetTrajectoryAction(trajectories.slalomPathStraightB.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
        // //5
        // runAction(new SetTrajectoryAction(trajectories.slalomPathEnd.get(true), 0.0, 1.0));
        // runAction(new WaitToFinishPathAction());
    }

}