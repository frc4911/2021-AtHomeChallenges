package frc.robot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.timing.TimedState;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.BallSearchAction;
import frc.robot.auto.actions.PathSelection;
import frc.robot.auto.actions.RemainingProgressAction;
import frc.robot.auto.actions.SetManualTrajectoryAction;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.LLTest;
import frc.robot.auto.actions.SetTrajectoryAction;
import frc.robot.auto.actions.WaitToFinishPathAction;
import frc.robot.auto.actions.setPoseAction;

public class TestMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        // runAction(new SetManualTrajectoryAction(getTrajectory(), 0.0, .50));
        
        runAction(new setPoseAction(new Pose2d(0,0,new Rotation2d(0.0))));



        runAction(new SetTrajectoryAction(trajectories.slalomPathA.get(true), -360.0, 0.6));
        // runAction(new RemainingProgressAction(0.5));

        // runAction(new SetTrajectoryAction(trajectories.slalomPathB.get(true), 0.0, 1.0));
        // runAction(new RemainingProgressAction(0.5));

        // runAction(new SetTrajectoryAction(trajectories.slalomPathC.get(true), 360.0, 1.0));
        // runAction(new RemainingProgressAction(0.5));

        // runAction(new SetTrajectoryAction(trajectories.slalomPathD.get(true), 0.0, 1.0));
        // runAction(new RemainingProgressAction(0.5));

        // runAction(new SetTrajectoryAction(trajectories.slalomPathE.get(true), 0.0, .50));
         runAction(new WaitToFinishPathAction());
    }

    private List<TimedState<Pose2dWithCurvature>> getTrajectory(){

        List<TimedState<Pose2dWithCurvature>> list = new ArrayList<TimedState<Pose2dWithCurvature>>();
               
        return list;
    }

}