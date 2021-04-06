package frc.robot.auto.actions;

import com.team254.lib.autos.actions.Action;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj.Timer;

public class ClearBallsAction implements Action {
	
	private String sClassName;
	private Superstructure mSuperstructure;
	private Indexer mIndexer;
    private boolean mTurnOn;
    private double mIndexerPos=0;
    private final double mIndexerTicksToEmpty = 50000;

	public ClearBallsAction() {
		sClassName = this.getClass().getSimpleName();
        mSuperstructure = Superstructure.getInstance(sClassName);
        mIndexer = Indexer.getInstance(sClassName);
	}
	
	@Override
	public boolean isFinished() {
		return mIndexer.getCurrentPosition() <= mIndexerPos-mIndexerTicksToEmpty;
	}
	
	@Override
	public void start() {
        mIndexerPos = mIndexer.getCurrentPosition();
        System.out.println("***** ClearBallsAction start - position:"+mIndexerPos+" ***** " + Timer.getFPGATimestamp());
        mSuperstructure.setWantedState(Superstructure.WantedState.CLEAR_BALLS);
	}
	
	@Override
	public void update() {
		
	}
	
	@Override
	public void done() {
		mSuperstructure.setWantedState(Superstructure.WantedState.HOLD);
        System.out.println("***** ClearBallsAction done  ***** " + Timer.getFPGATimestamp());
        
	}
}
