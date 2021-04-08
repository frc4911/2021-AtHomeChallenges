package frc.robot.auto.actions;

import com.team254.lib.autos.actions.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Indexer;

public class SetBallCountAction implements Action {
	
	private Indexer mIndexer;
	private int balls;

	public SetBallCountAction(int balls) {
		String sClassName = this.getClass().getSimpleName();
        mIndexer = Indexer.getInstance(sClassName);
        this.balls = balls;
	}
	
	@Override
	public boolean isFinished() {
		return true;
	}
	
	@Override
	public void start() {
        System.out.println("SetBallCountAction start - "+balls+" ("+Timer.getFPGATimestamp()+")");
        mIndexer.setNumberOfBalls(balls);
	}
	
	@Override
	public void update() {

	}
	
	@Override
	public void done() {
       System.out.println("SetBallCountAction done ("+Timer.getFPGATimestamp()+")");
	}
}