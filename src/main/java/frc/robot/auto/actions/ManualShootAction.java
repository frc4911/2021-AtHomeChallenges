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

	public ManualShootAction(double duration) {
		target = Timer.getFPGATimestamp() + duration;
	}
	
	public ManualShootAction(double duration, double rpm) {
		target = Timer.getFPGATimestamp() + duration;
	}
	
	public ManualShootAction(double duration, double rpm, int balls) {
		target = Timer.getFPGATimestamp() + duration;
		this.balls = balls;
		this.rpm = rpm;
	}
	
	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() >= target;
	}
	
	@Override
	public void start() {
		mSuperstructure.setManualShootRPM(rpm);
		System.out.println("action set superstructure wanted state to manual_shoot");
		mSuperstructure.setWantedState(Superstructure.WantedState.MANUAL_SHOOT);
	}
	
	@Override
	public void update() {

	}
	
	@Override
	public void done() {
		mSuperstructure.setWantedState(Superstructure.WantedState.HOLD);
	}	
}
