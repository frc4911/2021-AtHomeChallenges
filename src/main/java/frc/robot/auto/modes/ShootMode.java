package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.WaitToFinishPathAction;
import frc.robot.auto.actions.ManualShootAction;
import frc.robot.auto.actions.AimAction;
import frc.robot.auto.actions.CollectBallsAction;
import frc.robot.auto.actions.ClearBallsAction;
import frc.robot.auto.actions.WaitAction; 
import frc.robot.auto.actions.SetIdleShooterRPMAction; 
import frc.robot.auto.actions.ManualShootBallsAction;

public class ShootMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println();
        System.out.println("Shoot Auto Mode Running!!!");
            runAction(new SetIdleShooterRPMAction(1000));
        while (true) {
            runAction(new CollectBallsAction(3));
            //runAction(new WaitAction(5.0));
            //runAction(new ClearBallsAction());
            
            runAction(new AimAction());
            runAction(new ManualShootBallsAction(1000.0,1.0));
            // runAction(new SetTrajectoryAction(trajectories.powerPortBackwardPath.get(true), 0.0, 1.0));
            // runAction(new WaitToFinishPathAction());
            // runAction(new CollectAction(true));
            // runAction(new CollectAction(false));
            // runAction(new SetTrajectoryAction(trajectories.powerPortForwardPath.get(true), 0.0, 1.0));
            // runAction(new WaitToFinishPathAction());
        }
        // runAction(new SetTrajectoryAction(trajectories.backAwayFromLinePath.get(true), 0.0, 1.0));   
    }
}