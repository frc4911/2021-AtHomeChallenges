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

	public CollectBallsAction(int ballsToCollect) {
		sClassName = this.getClass().getSimpleName();
        mSuperstructure = Superstructure.getInstance(sClassName);
        mIndexer = Indexer.getInstance(sClassName);
        this.ballsToCollect = ballsToCollect;
	}
	
	@Override
	public boolean isFinished() {
		System.out.println("current num of balls: "+mIndexer.getNumberOfBalls());
        return mIndexer.getNumberOfBalls() >= ballTarget;
	}
	
	@Override
	public void start() {
        ballTarget = mIndexer.getNumberOfBalls()+ballsToCollect;
        ballTarget = Math.min(ballTarget,3);
        ballTarget = Math.max(ballTarget,0);
        System.out.println("CollectBallsAction start, current ball count:"+ mIndexer.getNumberOfBalls()+ " target ball count:"+ballTarget+" ("+Timer.getFPGATimestamp()+")");
    	mSuperstructure.setWantedState(Superstructure.WantedState.COLLECT);
	}
	
	@Override
	public void update() {
		
	}
	
	@Override
	public void done() {
		mSuperstructure.setWantedState(Superstructure.WantedState.HOLD);
        System.out.println("CollectBallsAction done, current ball count:"+ mIndexer.getNumberOfBalls()+" ("+Timer.getFPGATimestamp()+")");
	}
}
