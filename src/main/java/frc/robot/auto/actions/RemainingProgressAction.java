package frc.robot.auto.actions;

import frc.robot.subsystems.Swerve;
import com.team254.lib.autos.actions.Action;

/**
 * An action designed to wait until there is only a certain (specified) 
 * amount of time left before the completion of a trajectory.
 */
public class RemainingProgressAction implements Action{
    Swerve swerve;
    double targetProgress = 0.0;
    private String sClassName;

    public RemainingProgressAction(double targetProgress){
        sClassName = this.getClass().getSimpleName();
        swerve = Swerve.getInstance(sClassName);
        this.targetProgress = targetProgress;
    }

    @Override
    public boolean isFinished() {
        return swerve.getRemainingProgress() <= targetProgress;
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }
}
