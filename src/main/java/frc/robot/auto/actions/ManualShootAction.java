package frc.robot.auto.actions;

import com.team254.lib.autos.actions.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ManualShootAction implements Action {
	int balls = 3;
	double rpm = 3700;

	private Superstructure mSuperstructure = Superstructure.getInstance("ManualShootAction");
	private Shooter mShooter = Shooter.getInstance("ManualShootAction");
	private double target = 0.0;
	private double duration = 0.0;

	public ManualShootAction(double duration) {
		this.duration = duration;
	}
	
	public ManualShootAction(double duration, double rpm) {
		this.duration = duration;
	}
	
	public ManualShootAction(double duration, double rpm, int balls) {
		this.duration = duration;
		this.balls = balls;
		this.rpm = rpm;
		System.out.println("***** Manual Shooting constructor ***** " + Timer.getFPGATimestamp());

	}

	@Override
	public boolean isFinished() {
		if (Timer.getFPGATimestamp() >= target){
			System.out.println("***** Manual Shooting isFinished - ***** " + Timer.getFPGATimestamp());
			return true;
		}
		return false;
	}
	
	@Override
	public void start() {
		target = Timer.getFPGATimestamp() + duration;
		mSuperstructure.setManualShootRPM(rpm);
		System.out.println("***** Manual Shooting start ***** " + Timer.getFPGATimestamp());
		mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT);
	}
	
	@Override
	public void update() {
		// System.out.println("***** Manual Shooting update ***** " + Timer.getFPGATimestamp());

	}
	
	@Override
	public void done() {
		System.out.println("***** Manual Shooting done - ***** " + Timer.getFPGATimestamp());
		mSuperstructure.setWantedState(Superstructure.WantedState.HOLD);
	}	
}
