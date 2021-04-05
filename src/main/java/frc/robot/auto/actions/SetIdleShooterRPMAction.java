package frc.robot.auto.actions;

import com.team254.lib.autos.actions.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Shooter;

public class SetIdleShooterRPMAction implements Action {
	
	private Shooter mShooter;
    private double rpm;

	public SetIdleShooterRPMAction(double rpm) {
		String sClassName = this.getClass().getSimpleName();
        mShooter = Shooter.getInstance(sClassName);
        this.rpm = rpm;
	}
	
	@Override
	public boolean isFinished() {
        return mShooter.getCurrentRPM() > rpm*.9;
	}
	
	@Override
	public void start() {
        System.out.println("SetIdleShooterRPMAction start - idle RPM:"+rpm+" ("+Timer.getFPGATimestamp()+")");
        mShooter.setHoldRPM(rpm);
	}
	
	@Override
	public void update() {

	}
	
	@Override
	public void done() {
        System.out.println("SetIdleShooterRPMAction done - current RPM:"+mShooter.getCurrentRPM()+" ("+Timer.getFPGATimestamp()+")");
	}
}