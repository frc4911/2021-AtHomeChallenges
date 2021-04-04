package frc.robot.auto.actions;

import com.team254.lib.autos.actions.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ManualShootBallsAction implements Action {
	
    private Superstructure mSuperstructure;
	private Indexer mIndexer;
	private Shooter mShooter;
	private double rpm;

	public ManualShootBallsAction(double rpm) {
		String sClassName = this.getClass().getSimpleName();
        mSuperstructure = Superstructure.getInstance(sClassName);
		mIndexer = Indexer.getInstance(sClassName);
		mShooter = Shooter.getInstance(sClassName);
		this.rpm = rpm;
	}
	
	@Override
	public boolean isFinished() {
		return mIndexer.getNumberOfBalls() <= 0;
	}
	
	@Override
	public void start() {
		System.out.println("ManualShootBallsAction start, current ball count:"+ mIndexer.getNumberOfBalls()+" ("+Timer.getFPGATimestamp()+")");
		mSuperstructure.setManualShootRPM(rpm);
		mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT);
	}
	
	@Override
	public void update() {

	}
	
	@Override
	public void done() {
		mShooter.setHoldRPM(rpm);
		mSuperstructure.setWantedState(Superstructure.WantedState.HOLD);
        System.out.println("ManualShootBallsAction done ("+Timer.getFPGATimestamp()+")");
	}
}