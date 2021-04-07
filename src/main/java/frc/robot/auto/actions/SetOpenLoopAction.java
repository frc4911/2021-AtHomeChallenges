package frc.robot.auto.actions;

import com.team254.lib.autos.actions.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

public class SetOpenLoopAction implements Action {    
    private Swerve mSwerve;
    private double speed = 0.0;
    private double duration = 0.0;
    private double target = 0.0;
    private String sClassName;

	public SetOpenLoopAction(double speed, double duration) {
        this.speed = speed; 
        this.duration = duration;
        sClassName = this.getClass().getSimpleName();;
        mSwerve = Swerve.getInstance(sClassName);
	}

	@Override
	public boolean isFinished() {
		if (Timer.getFPGATimestamp() >= target){
			System.out.println("***** Set Open Loop Action isFinished - ***** " + Timer.getFPGATimestamp());
			return true;
		}
		return false;
	}
	
	@Override
	public void start() {
		target = Timer.getFPGATimestamp() + duration;
        System.out.println("***** Set Open Loop Action start ***** " + Timer.getFPGATimestamp());
        mSwerve.sendInput(speed, 0, 0, false, false);
	}
	
	@Override
	public void update() {
		// System.out.println("***** Manual Shooting update ***** " + Timer.getFPGATimestamp());

	}
	
	@Override
	public void done() {
		System.out.println("***** Set Open Loop Action done - ***** " + Timer.getFPGATimestamp());
		mSwerve.setState(Swerve.ControlState.NEUTRAL);
	}	
}
