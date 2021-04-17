
package frc.robot.auto.actions;

import frc.robot.subsystems.Swerve;
import com.team254.lib.autos.actions.Action;
import com.team254.lib.geometry.Pose2d;

public class setPoseAction implements Action{
    Swerve swerve;
    Pose2d pose;
    private String sClassName;

    public setPoseAction(Pose2d pose){
        sClassName = this.getClass().getSimpleName();
        swerve = Swerve.getInstance(sClassName);
        this.pose = pose;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void start() {
        swerve.zeroSensors(pose);
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }
}
