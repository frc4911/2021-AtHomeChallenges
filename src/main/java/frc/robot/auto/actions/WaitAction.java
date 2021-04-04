package frc.robot.auto.actions;

import com.team254.lib.autos.actions.Action;

import edu.wpi.first.wpilibj.Timer;

public class WaitAction implements Action {
	
	private double target = 0.0;
    private double duration = 0.0;

	public WaitAction(double duration) {
		this.duration = duration;
	}
	
	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() >= target;
	}
	
	@Override
	public void start() {
        System.out.println("***** Waiting - " + duration + " ***** " + Timer.getFPGATimestamp());
		target = Timer.getFPGATimestamp() + duration;
	}
	
	@Override
	public void update() {
        
	}
	
	@Override
	public void done() {
		System.out.println("***** Done Waiting - " + duration + " ***** " + Timer.getFPGATimestamp());
	}	
}
