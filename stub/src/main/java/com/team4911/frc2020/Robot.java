/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team4911.frc2020;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
//import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;

    // WPI_TalonSRX m_LeftFrontTalon = new WPI_TalonSRX(Constants.LEFT_DRIVE_FRONT_TALON_PORT);
    // WPI_TalonSRX m_LeftRearTalon = new WPI_TalonSRX(Constants.LEFT_DRIVE_REAR_TALON_PORT);
    // SpeedControllerGroup m_LeftDrive = new SpeedControllerGroup(this.m_LeftFrontTalon, this.m_LeftRearTalon);

    // WPI_TalonSRX m_RightFrontTalon = new WPI_TalonSRX(Constants.RIGHT_DRIVE_FRONT_TALON_PORT);
    // WPI_TalonSRX m_RightRearTalon = new WPI_TalonSRX(Constants.RIGHT_DRIVE_REAR_TALON_PORT);
    // SpeedControllerGroup m_RightDrive = new SpeedControllerGroup(this.m_RightFrontTalon, this.m_RightRearTalon);

    // DifferentialDrive m_DriveTrain = new DifferentialDrive(m_LeftDrive, m_RightDrive);

    // Joystick m_LeftJoystick = new Joystick(Constants.LEFT_JOYSTICK_PORT);
    // Joystick m_RightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_PORT);

    String manifestStr = "There was a problem reading the manifest";
    int loopCnt = 0;

    TalonSRX motor15 = new TalonSRX(15);
    TalonSRX motor16 = new TalonSRX(16);
    TalonSRX motor17 = new TalonSRX(17);

    // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // NetworkTableEntry tv = table.getEntry("tv");
    // NetworkTableEntry tx = table.getEntry("tx");
    // NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        // this.m_LeftFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);
        // this.m_RightFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100);

    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {

    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }
    int jj=0;
    @Override
    public void teleopInit() {
        System.out.println("stub bbb");
    }
    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

        motor15.set(ControlMode.PercentOutput,.5);
        motor16.set(ControlMode.PercentOutput,.5);
        motor17.set(ControlMode.PercentOutput,.5);
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    public void disabledInit(){
        motor15.set(ControlMode.PercentOutput,0);
        motor16.set(ControlMode.PercentOutput,0);
        motor17.set(ControlMode.PercentOutput,0);
    }
}
