/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team4911.frc2020;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.lib.drivers.LazyTalonSRX;

import cyberlib.io.CW;
import cyberlib.io.LogitechPS4;
import cyberlib.io.Thrustmaster;
import cyberlib.io.Xbox;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
//import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	Xbox driver = null;

	@Override
	public void robotInit() {
		SmartDashboard.putString("State","robotInit");
		driver = new Xbox();
//		driver = new LogitechPS4();
//		driver = new Thrustmaster();
		createMotors();
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
	}
	double speed = 0;
	double kp = 0;
	double ki = 0;
	double kd = 0;
	double kf = 0;

	@Override
	public void teleopInit() {
		System.out.println("teleopInit");
		driver.rumble(1, 10);

		SmartDashboard.putString("State","teleopInit");

		speed = SmartDashboard.getNumber("wheel speed", 0);
		SmartDashboard.putNumber("wheel speed", speed);

		kp = SmartDashboard.getNumber("wheel kp", 0);
		SmartDashboard.putNumber("wheel kp", kp);

		ki = SmartDashboard.getNumber("wheel ki", 0);
		SmartDashboard.putNumber("wheel ki", ki);

		kd = SmartDashboard.getNumber("wheel kd", 0);
		SmartDashboard.putNumber("wheel kd", kd);

		kf = SmartDashboard.getNumber("wheel kf", 0);
		SmartDashboard.putNumber("wheel kf", kf);

		// leftmaster0.config_kP(0,kp);
		// leftmaster0.config_kI(0,ki);
		// leftmaster0.config_kD(0,kd);
		// leftmaster0.config_kF(0,kf);
		// System.out.print("teleopInit");
	}
	
	@Override
	public void teleopPeriodic() {
		if(driver.getButton(Xbox.A_BUTTON,CW.PRESSED_EDGE)){
			System.out.println("A pressededge");
		}
		if(driver.getButton(Xbox.A_BUTTON,CW.RELEASED_EDGE)){
			System.out.println("A releasededge");
		}
		if(driver.getButton(Xbox.A_BUTTON,CW.PRESSED_LEVEL_LONG)){
			System.out.println("A pressedlevellong");
		}
		if(driver.getButton(Xbox.A_BUTTON,CW.RELEASED_EDGE_QUICK)){
			System.out.println("A releasededgequick");
		}
		if(driver.getButton(Xbox.A_BUTTON,CW.RELEASED_EDGE_LONG)){
			System.out.println("A releasededgelong");
		}
		if(driver.getButton(Xbox.POV0_0,CW.PRESSED_EDGE)){
			System.out.println("POV_0 pressededge");
		}
		if(driver.getButton(Xbox.POV0_0,CW.RELEASED_EDGE)){
			System.out.println("POV_0 releasededge");
		}
		if(driver.getButton(Xbox.POV0_0,CW.PRESSED_LEVEL_LONG)){
			System.out.println("POV_0 pressedlevellong");
		}
		if(driver.getButton(Xbox.POV0_0,CW.RELEASED_EDGE_QUICK)){
			System.out.println("POV_0 releasededgequick");
		}
		if(driver.getButton(Xbox.POV0_0,CW.RELEASED_EDGE_LONG)){
			System.out.println("POV_0 releasededgelong");
		}
		if(driver.getButton(Xbox.POV0_180,CW.RELEASED_EDGE_LONG)){
			System.out.println("POV_0 releasededgelong");
		}
		if(driver.getRaw(Xbox.POV0_180)==180.0){
			System.out.println("POV_0 raw is 180");
		}


		//if(driver.getButtonA_getRaw()==1.0){
            // leftmaster0.set(ControlMode.Velocity,speed);
		//}
		//else {
		//	runMotors(driver.getRightStickY());
		//}

		// SmartDashboard.putNumber("Left Master Vel", leftmaster0.getSelectedSensorVelocity());
		// SmartDashboard.putNumber("Left Master Current", leftmaster0.getStatorCurrent());
	}

	@Override
	public void disabledInit() {
		SmartDashboard.putString("State","disabledInit");
		// System.out.print("disabledInit");
	}

	@Override
	public void disabledPeriodic() {
		// System.out.print("disabledPeriodic");
		stopMotors();
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}

	TalonSRX falconR = null;
	// TalonSRX leftmaster0 = null;
	// TalonSRX leftslave1 = null;
	// TalonSRX leftslave2 = null;
	// TalonSRX s2 = null;
	// LazySparkMax s3 = null;
	// LazySparkMax s4 = null;

	private void createMotors(){

		falconR = new LazyTalonSRX(21);
		// leftmaster0 = new LazyTalonSRX(0);
		// leftslave1 = new TalonSRX(1);
		// leftslave2 = new TalonSRX(2);

		// leftmaster0.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
		// leftmaster0.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
		// leftslave1.follow(leftmaster0);
		// leftslave2.follow(leftmaster0);
		// 	s2 = new TalonSRX(21);
	 	// s3 = new LazySparkMax(22);
	// 	s4 = new LazySparkMax(23);
	}

	private void runMotors(double speed){

	// 	double ss1 = SmartDashboard.getNumber("ss1", 0);
	// 	if (ss1 == 0){
	// 		SmartDashboard.putNumber("ss1", 0);
	// 	}
	// 	double ss2 = SmartDashboard.getNumber("ss2", 0);
	// 	if (ss2 == 0){
	// 		SmartDashboard.putNumber("ss2", 0);
	// 	}
	// 	double ss3 = SmartDashboard.getNumber("ss3", 0);
	// 	if (ss3 == 0){
	// 		SmartDashboard.putNumber("ss3", 0);
	// 	}
	// 	double ss4 = SmartDashboard.getNumber("ss4", 0);
	// 	if (ss4 == 0){
	// 		SmartDashboard.putNumber("ss4", 0);
	// 	}
	// leftmaster0.set(ControlMode.PercentOutput,speed);
	falconR.set(ControlMode.PercentOutput,speed);
	// 	s2.set(ControlMode.PercentOutput,ss2);
	// 	s3.set(ss3);
	// 	s4.set(ss4);
	// 	//SmartDashboard.putNumber("shooterV",)
	}
	
	private void stopMotors(){
		// leftmaster0.set(ControlMode.PercentOutput,0);
		falconR.set(ControlMode.PercentOutput,0);
	// 	s2.set(ControlMode.PercentOutput,0);
	// 	s3.set(0);
	// 	s4.set(0);

	}
	
}
