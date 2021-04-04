package frc.robot.auto.actions;

import com.team254.lib.autos.actions.Action;
import frc.robot.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;

public class CollectAction implements Action {
	
	private String sClassName;
	private Superstructure mSuperstructure;
	private boolean mTurnOn;

	public CollectAction(boolean turnOn) {
		sClassName = this.getClass().getSimpleName();
		mSuperstructure = Superstructure.getInstance(sClassName);
		mTurnOn = turnOn;
	}
	
	@Override
	public boolean isFinished() {
		return true;
	}
	
	@Override
	public void start() {
		System.out.println("***** Collecting - " + mTurnOn + " ***** " + Timer.getFPGATimestamp());
		if (mTurnOn) {
			mSuperstructure.setWantedState(Superstructure.WantedState.COLLECT);
		} else {
			mSuperstructure.setWantedState(Superstructure.WantedState.HOLD);
		}
	}
	
	@Override
	public void update() {

	}
	
	@Override
	public void done() {
		System.out.println("***** Done Collecting - " + mTurnOn + " ***** " + Timer.getFPGATimestamp());
	}
}
