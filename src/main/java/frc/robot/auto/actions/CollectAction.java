package frc.robot.auto.actions;

import com.team254.lib.autos.actions.Action;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Indexer;
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
		// if (turnOn) {
		// 	return getNumberOfBalls() == 3;
		// }else{
			return true;
		//}
	}
	
	@Override
	public void start() {
		System.out.println("***** Collecting - " + mTurnOn + " ***** " + Timer.getFPGATimestamp());
		if (mTurnOn) {
			mSuperstructure.setWantedState(Superstructure.WantedState.COLLECT, sClassName);
		} else {
			mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
		}
	}
	
	@Override
	public void update() {

	}
	
	@Override
	public void done() {
        System.out.println(sClassName+" ***** done  ***** ("+Timer.getFPGATimestamp()+")");
	}
}
