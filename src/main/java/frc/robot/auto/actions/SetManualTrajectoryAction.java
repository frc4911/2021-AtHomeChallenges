package frc.robot.auto.actions;

import java.util.List;

import com.team254.lib.autos.actions.RunOnceAction;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import frc.robot.subsystems.Swerve;

public class SetManualTrajectoryAction extends RunOnceAction {
	Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
    double goalHeading;
    double rotationScalar;
	Swerve swerve;
	
	// public SetManualTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double goalHeading, double rotationScalar){
    public SetManualTrajectoryAction(List<TimedState<Pose2dWithCurvature>> list, double goalHeading, double rotationScalar){
        trajectory = new Trajectory<TimedState<Pose2dWithCurvature>>(list);
        this.goalHeading = goalHeading;
        this.rotationScalar = rotationScalar;
		swerve = Swerve.getInstance("SetManualTrajectoryAction");
	}
	
	@Override
	public synchronized void runOnce(){
		swerve.setTrajectory(trajectory, goalHeading, rotationScalar);
	}
}

