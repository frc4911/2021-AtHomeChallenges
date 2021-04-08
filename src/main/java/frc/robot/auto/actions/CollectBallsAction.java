package frc.robot.auto.actions;

import com.team254.lib.autos.actions.Action;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj.Timer;

public class CollectBallsAction implements Action {
	
	private String sClassName;
    private Superstructure mSuperstructure;
    private Indexer mIndexer;
    private int ballTarget;
	private int ballsToCollect;
	private int lastBallCount;

	public CollectBallsAction(int ballsToCollect) {
		sClassName = this.getClass().getSimpleName();
        mSuperstructure = Superstructure.getInstance(sClassName);
        mIndexer = Indexer.getInstance(sClassName);
        this.ballsToCollect = ballsToCollect;
	}
	
	@Override
	public boolean isFinished() {
		int currentBallCount = mIndexer.getNumberOfBalls();
		if (currentBallCount != lastBallCount){
			System.out.println("currentBallCount is "+currentBallCount);
			lastBallCount = currentBallCount;
		}
        return mIndexer.getNumberOfBalls() >= ballTarget;
	}
	
	@Override
	public void start() {
		lastBallCount = mIndexer.getNumberOfBalls();
        ballTarget = lastBallCount+ballsToCollect;
        ballTarget = Math.min(ballTarget,3);
        ballTarget = Math.max(ballTarget,0);
        System.out.println(sClassName+" start, current ball count:"+ lastBallCount+ " target ball count:"+ballTarget+" ("+Timer.getFPGATimestamp()+")");
    	mSuperstructure.setWantedState(Superstructure.WantedState.COLLECT, sClassName);
	}
	
	@Override
	public void update() {
		
	}
	
	@Override
	public void done() {
		mSuperstructure.setWantedState(Superstructure.WantedState.HOLD, sClassName);
        System.out.println(sClassName+" ***** done  ***** ("+Timer.getFPGATimestamp()+")");
        System.out.println("current ball count:"+ mIndexer.getNumberOfBalls());
	}
}
