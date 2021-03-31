package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.subsystems.SubsystemManager;

import cyberlib.utils.CheckFaults;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.auto.SmartDashboardInteractions;

public class Indexer extends Subsystem {

    // Hardware
    private final TalonFX mFXLeft, mFXRight;
    private final DigitalInput mDIBallEntered;         // true == no ball, false == ball
    private final DigitalInput mDIBallTouchingShooter; // true == no ball, false == ball
    private final AnalogInput mAIBallEntered; //Port 1
    private final AnalogInput mAIBallTouchingShooter; //Port 0
    private final Solenoid mSolPTO;           // true == climber, false == indexer

    private double mFXLeftPIDPos;
    private double mFXRightPIDPos;

    private final double kIndexSpeed = 0.7;
    private final double kLoadSpeed = 0.25; // .5 brian
    private final double kBackSpeed = -0.1;
    private final int beamBreakThreshold = 3;
    private PTO mPTOState;
    private double mClimberPrepDemand = .15;
    private double mClimberPrepCnt = 0; // TODO: change to be Timer based
    
    private double deltaFromEntrance = 3652.0; //Encoder ticks wanted after ball passing beam break *****************************************/
    private double startBallPos = 0.0; //Position right as the ball leaves beam break *******************************************************/

    public enum PTO {
        CLIMBER(true),
        INDEXER(false);

        private final boolean state;

        private PTO(boolean state){
            this.state = state;
        }

        public boolean get(){
            return state;
        }
    }

    public enum SystemState {
        HOLDING,
        LOADING,
        CORRECTING,
        BACKING,
        INDEXING,
        CLIMBING_PREP,
        CLIMBING_MOVE
    }

    public enum WantedState {
        HOLD,
        LOAD,
        CORRECT,
        BACK,
        INDEX,
        CLIMB_PREP,
        CLIMB_MOVE
    }

    private SystemState      mSystemState = SystemState.HOLDING;
    private WantedState      mWantedState = WantedState.HOLD;
    private PeriodicIO       mPeriodicIO;
    private boolean          mStateChanged;
    private CheckFaults      mCF = new CheckFaults();
    private final boolean    mLoggingEnabled = true; // used to disable logging for this subsystem only
    private int              mDefaultSchedDelta = 20;
    private int              mListIndex;
    private SubsystemManager mSubsystemManager;

    private static String sClassName;
    private static int sInstanceCount;
    private static Indexer sInstance = null;
    public  static Indexer getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Indexer(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Indexer(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mPeriodicIO = new PeriodicIO();
        mFXLeft = TalonFXFactory.createDefaultTalon(Ports.INDEXER_LEFT);
        mFXRight = TalonFXFactory.createDefaultTalon(Ports.INDEXER_RIGHT);
        mSolPTO = new Solenoid(0, Ports.POWER_TAKE_OFF);
        mDIBallEntered = new DigitalInput(Ports.ENTRANCE_BEAM_BREAK);
        mDIBallTouchingShooter = new DigitalInput(Ports.EXIT_BEAM_BREAK);
        mAIBallEntered = new AnalogInput(Ports.ENTERANCE_BEAM_BREAK_A);
        mAIBallTouchingShooter = new AnalogInput(Ports.EXIT_BEAM_BREAK_A);
        mSubsystemManager = SubsystemManager.getInstance(sClassName);
        configMotors();
    }

    private void configMotors() {
        mFXLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);
        mFXRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs);

        mFXLeft.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXRight.configForwardSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXLeft.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);
        mFXRight.configReverseSoftLimitEnable(false, Constants.kLongCANTimeoutMs);

        mFXLeft.setInverted(false);
        mFXRight.setInverted(false);

        mFXLeft.setSensorPhase(false);
        mFXRight.setSensorPhase(false);

        // PID to hold climber until brake is engaged
        // TODO: change dynamically based on system state
        mFXLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 19, Constants.kLongCANTimeoutMs);
        mFXRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 19, Constants.kLongCANTimeoutMs);

        mFXLeft.configAllowableClosedloopError(0, 0, Constants.kLongCANTimeoutMs);
    	mFXRight.configAllowableClosedloopError(0, 0, Constants.kLongCANTimeoutMs);

        mFXLeft.selectProfileSlot(0, 0);
        mFXRight.selectProfileSlot(0, 0);
        
        double kp = 0.45;
        double ki = 0.0;
        double kd = 0.0;
        double kf = 0.0;

		mFXLeft.config_kP(0, kp, Constants.kLongCANTimeoutMs);
    	mFXLeft.config_kI(0, ki, Constants.kLongCANTimeoutMs);
    	mFXLeft.config_kD(0, kd, Constants.kLongCANTimeoutMs);
		mFXLeft.config_kF(0, kf, Constants.kLongCANTimeoutMs);

		mFXRight.config_kP(0, kp, Constants.kLongCANTimeoutMs);
    	mFXRight.config_kI(0, ki, Constants.kLongCANTimeoutMs);
    	mFXRight.config_kD(0, kd, Constants.kLongCANTimeoutMs);
		mFXRight.config_kF(0, kf, Constants.kLongCANTimeoutMs);

        // current limit to 40 amp
        // StatorCurrentLimitConfiguration sclc = new StatorCurrentLimitConfiguration(true,50,50,0);
        // mFXLeft.configStatorCurrentLimit(sclc); // enabled, continuous, current threshold, timeout threshold
        // mFXRight.configStatorCurrentLimit(sclc); // enabled, continuous, current threshold, timeout threshold

        SupplyCurrentLimitConfiguration sclc = new SupplyCurrentLimitConfiguration(true,50,50,0);
        mFXLeft.configSupplyCurrentLimit(sclc);
        mFXRight.configSupplyCurrentLimit(sclc);

        setNeutralMode(NeutralMode.Brake);
    }

    private void setNeutralMode(NeutralMode mode) {
        mFXLeft.setNeutralMode(mode);
        mFXRight.setNeutralMode(mode);
    }

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(Phase phase) {
            synchronized(Indexer.this) {
                mSystemState = SystemState.HOLDING;
                mWantedState = WantedState.HOLD;
                mStateChanged = true;
                System.out.println(sClassName + " state " + mSystemState);
                mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                stop();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(Indexer.this) {
                SystemState newState;
                switch(mSystemState) {
                    case LOADING:
                        newState = handleLoading();
                        break;
                    case CORRECTING:
                        newState = handleCorrecting();
                        break;
                    case BACKING:
                        newState = handleBacking();
                        break;
                    case INDEXING:
                        newState = handleIndexing();
                        break;
                    case CLIMBING_PREP:
                    case CLIMBING_MOVE:
                        newState = handleClimbing();
                        break;
                    case HOLDING:
                    default:
                        newState = handleHolding();
                }

                if (newState != mSystemState) {
                    System.out.println(sClassName + " state " + mSystemState + " to " + newState + " (" + timestamp + ")");
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }

            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private SystemState handleHolding() {
        if (mStateChanged) {
            mPeriodicIO.indexerDemand = 0.0;
            mPeriodicIO.climberDemand = 0.0;
            mPeriodicIO.PTODemand = PTO.INDEXER;  // PTO is always indexer unless climbing
            mPeriodicIO.schedDeltaDesired = 0; // goto sleep
            System.out.println("holdingEncPos - " + mPeriodicIO.FXLeftEncPos);
        }
        
        return defaultStateTransfer();
    }

    private SystemState handleLoading() {
        if (mStateChanged) {
            double speed = SmartDashboard.getNumber("indexer speed",-1);
            if (speed == -1){
                SmartDashboard.putNumber("indexer speed",kLoadSpeed);
                speed = kLoadSpeed;
            }
            mPeriodicIO.indexerDemand = speed; //kLoadSpeed;
            mPeriodicIO.PTODemand = PTO.INDEXER;
            mPeriodicIO.schedDeltaDesired = 20; // goto sleep
        }else if (!isBallEntering()){
            setWantedState(WantedState.CORRECT);
            startBallPos = mPeriodicIO.FXLeftEncPos;
        }

        return defaultStateTransfer();
    }

    private SystemState handleCorrecting() {
        if (mStateChanged) {            
            deltaFromEntrance = SmartDashboard.getNumber("delta from start",-1);
            if (deltaFromEntrance == -1){
                SmartDashboard.putNumber("delta from start", 3562);
                deltaFromEntrance = 3562;
            }
            mPeriodicIO.indexerPosDemand = startBallPos + deltaFromEntrance;
            mPeriodicIO.indexerDemand = 0;
            mPeriodicIO.schedDeltaDesired = 20;
            //System.out.println("startBallPos - " + startBallPos);
        }
        System.out.println("delta from start - " + deltaFromEntrance);
        System.out.println("indexerPosDemand - " + mPeriodicIO.indexerPosDemand + ", currentPos - " + mPeriodicIO.FXLeftEncPos + ", difference = " + Math.abs(mPeriodicIO.FXLeftEncPos - mPeriodicIO.indexerPosDemand));
        if (Math.abs(mPeriodicIO.FXLeftEncPos - mPeriodicIO.indexerPosDemand) < 100) {
            //System.out.println("Done Correcting");
            setWantedState(WantedState.HOLD);
        }else{
            //System.out.println("Not Done Correcting");
        }
        return defaultStateTransfer();
    }

    private SystemState handleBacking() {
        if (mStateChanged) {
            mPeriodicIO.indexerDemand = kBackSpeed;
            mPeriodicIO.PTODemand = PTO.INDEXER;
            mPeriodicIO.schedDeltaDesired = 0; // goto sleep
        }

        return defaultStateTransfer();
    }

    private SystemState handleIndexing() {
        if (mStateChanged) {
            mPeriodicIO.indexerDemand = kIndexSpeed;
            mPeriodicIO.PTODemand = PTO.INDEXER;
            mPeriodicIO.schedDeltaDesired = 0; // goto sleep
        }

        return defaultStateTransfer();
    }

    private SystemState handleClimbing() {
        if (mStateChanged) {
            mPeriodicIO.climberDemand = 0.0;
            mPeriodicIO.PTODemand = PTO.CLIMBER;
            mPeriodicIO.schedDeltaDesired = mDefaultSchedDelta; // stay awake
        }

        return defaultStateTransfer();
    }   

    public synchronized void setClimberSpeed(double speed) {
        mPeriodicIO.climberDemand = speed;
    }

    // this needs more thought
    // it needs to have a dedicated system state
    // public synchronized void setIndexerSpeed(double speed) {
        // mPeriodicIO.indexerDemand = speed;
    // }

    // method needs to be called at different cycle times
    public synchronized boolean isBallEntering() {
        //return !mDIBallEntered.get();
        return (mAIBallEntered.getVoltage() < beamBreakThreshold);

    }

    // method needs to be called at different cycle times
    public synchronized boolean isFullyLoaded() {
        //return !mDIBallTouchingShooter.get();
        return (mAIBallTouchingShooter.getVoltage() < beamBreakThreshold);

    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case LOAD:
                return SystemState.LOADING;
            case CORRECT:
                return SystemState.CORRECTING;
            case BACK:
                return SystemState.BACKING;
            case INDEX:
                return SystemState.INDEXING;
            case CLIMB_PREP:
                return SystemState.CLIMBING_PREP;
            case CLIMB_MOVE:
                return SystemState.CLIMBING_MOVE;
            case HOLD:
            default:
                return SystemState.HOLDING;
        }
    }

    public synchronized void setWantedState(WantedState state) {
        if (state != mWantedState) {
            mSubsystemManager.scheduleMe(mListIndex, 1, true);
            System.out.println("waking " + sClassName);
        }

        mWantedState = state;
    }

    public synchronized WantedState getWantedState(){
        return mWantedState;
    }

    @Override
    public void stop() {
        mFXLeft.set(ControlMode.PercentOutput, 0.0);
        mFXRight.set(ControlMode.PercentOutput, 0.0);
        mPeriodicIO.indexerDemand = 0;
        mPeriodicIO.climberDemand = 0;

        mSolPTO.set(PTO.INDEXER.get());  
        mPeriodicIO.PTODemand = PTO.INDEXER;
        mPTOState = PTO.INDEXER;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(mLoop);
    }
    
    @Override
    public String getLogHeaders() {
        if (mLoggingEnabled){
            mCF.clearFaults(mFXLeft);
            mCF.clearFaults(mFXRight);
            return  sClassName+".systemState,"+
                    sClassName+".FXEncLeftPos,"+
                    sClassName+".FXEncRightPos,"+
                    sClassName+".isBallEntering,"+
                    sClassName+".isFullyLoaded,"+
                    sClassName+".indexerDemand,"+
                    sClassName+".climberDemand,"+
                    sClassName+".powerTakeOffDemand,"+
                    sClassName+".FXLeftCurrent,"+
                    sClassName+".FXRightCurrent,"+
                    sClassName+".FXLeftMotorFault,"+
                    sClassName+".FXRightMotorFault,"+
                    sClassName+".schedDeltaDesired,"+
                    sClassName+".schedDeltaActual,"+
                    sClassName+".schedDuration";
        }
        return null;
    }

    private String generateLogValues(boolean telemetry){
        String values;
        if (telemetry){
            mPeriodicIO.FXLeftCurrent = mFXLeft.getStatorCurrent();
            mPeriodicIO.FXRightCurrent = mFXRight.getStatorCurrent();
            mPeriodicIO.FXLeftFaults = mCF.getFaults(mFXLeft);
            mPeriodicIO.FXRightFaults = mCF.getFaults(mFXRight);

            values = ""+mSystemState + "," +
                        /*mPeriodicIO.FXLeftEncPos+*/","+
                        /*mPeriodicIO.FXRightEncPos+*/","+
                        isBallEntering()+","+
                        isFullyLoaded()+","+
                        mPeriodicIO.indexerDemand+","+
                        mPeriodicIO.climberDemand+","+
                        mPeriodicIO.PTODemand+","+
                        mPeriodicIO.FXLeftCurrent+","+
                        mPeriodicIO.FXRightCurrent+","+
                        mPeriodicIO.FXLeftFaults+","+
                        mPeriodicIO.FXRightFaults+","+
                        /*mPeriodicIO.schedDeltaDesired+*/","+
                        /*mPeriodicIO.schedDeltaActual+*/","
                        /*mPeriodicIO.schedDuration*/;
    
        }
        else {
            mPeriodicIO.schedDuration = Timer.getFPGATimestamp() - mPeriodicIO.lastSchedStart;
            values = ""+mSystemState + "," +
                        mPeriodicIO.FXLeftEncPos+","+
                        mPeriodicIO.FXRightEncPos+","+
                        isBallEntering()+","+
                        isFullyLoaded()+","+
                        mPeriodicIO.indexerDemand+","+
                        mPeriodicIO.climberDemand+","+
                        mPeriodicIO.PTODemand+","+
                        /*mPeriodicIO.FXLeftCurrent+*/","+
                        /*mPeriodicIO.FXRightCurrent+*/","+
                        /*mPeriodicIO.FXLeftFaults+*/","+
                        /*mPeriodicIO.FXRightFaults+*/","+
                        mPeriodicIO.schedDeltaDesired+","+
                        mPeriodicIO.schedDeltaActual+","+
                        mPeriodicIO.schedDuration;
    
        }
        return values;
    }

    @Override
    public String getLogValues(boolean telemetry) {
        if (mLoggingEnabled){
            return generateLogValues(telemetry);
        }
        return null;
    }


    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;
        //if (mSystemState == SystemState.CLIMBING_PREP || mSystemState == SystemState.CLIMBING_MOVE) {
            mPeriodicIO.FXLeftEncPos  = mFXLeft.getSelectedSensorPosition();
            mPeriodicIO.FXRightEncPos = mFXRight.getSelectedSensorPosition();
        //}
        // if(mPeriodicIO.currentPos == mFXLeft.getSelectedSensorPosition()){
        //     indexerMoving = false;
        // }else{
        //     indexerMoving = true;
        // }
        // mPeriodicIO.currentPos = mFXLeft.getSelectedSensorPosition();
     }

    @Override
    public void writePeriodicOutputs() {
        if (mSystemState == SystemState.CLIMBING_PREP){
            // toggle motor back and forth to help free brake
            mFXLeft.set(ControlMode.PercentOutput, mClimberPrepDemand);
            mFXRight.set(ControlMode.PercentOutput, mClimberPrepDemand);
            if (mClimberPrepCnt++ % 5 == 0)    {
                mClimberPrepDemand *= -1;
            }
            mFXLeftPIDPos = Integer.MIN_VALUE;
        }
        else if (mSystemState == SystemState.CLIMBING_MOVE){
            // holding PID if motion stops
            if (mPeriodicIO.climberDemand == 0){
                if (mFXLeftPIDPos == Integer.MIN_VALUE){
                    mFXLeftPIDPos = mPeriodicIO.FXLeftEncPos;
                    mFXRightPIDPos = mPeriodicIO.FXRightEncPos;
                    mFXLeft.set(ControlMode.Position, mFXLeftPIDPos);
                    mFXRight.set(ControlMode.Position, mFXRightPIDPos);    
                }
            }
            else {
                // move based on joystick
                mFXLeftPIDPos = Integer.MIN_VALUE;
                mFXLeft.set(ControlMode.PercentOutput, mPeriodicIO.climberDemand);
                mFXRight.set(ControlMode.PercentOutput, mPeriodicIO.climberDemand);
            }
        } 
        else {

            System.out.println(mSystemState);
            if(mSystemState == SystemState.CORRECTING){
                mFXLeft.set(ControlMode.Position, mPeriodicIO.indexerPosDemand);
                mFXRight.set(ControlMode.Position, mPeriodicIO.indexerPosDemand);
            }else{
                mFXLeft.set(ControlMode.PercentOutput, mPeriodicIO.indexerDemand);
                mFXRight.set(ControlMode.PercentOutput, mPeriodicIO.indexerDemand);
            }
        }

        // control PTO
        // if (mPTOState != mPeriodicIO.PTODemand) {
        //     mPTOState = mPeriodicIO.PTODemand;
        //     mSolPTO.set(mPeriodicIO.PTODemand.get());
        // }
    }

    @Override
    public int whenRunAgain () {
        if (mStateChanged && mPeriodicIO.schedDeltaDesired == 0) {
            return 1;
        }
        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Ball Entered", isBallEntering());
        SmartDashboard.putBoolean("Fully Loaded", isFullyLoaded());
        SmartDashboard.putNumber("Enterance Beam Break Analog", mAIBallEntered.getVoltage());
        SmartDashboard.putNumber("Exit Beam Break Analog", mAIBallTouchingShooter.getVoltage());
        SmartDashboard.putNumber("Left Indexer Enc", mPeriodicIO.FXLeftEncPos);
        SmartDashboard.putNumber("Right Indexer Enc", mPeriodicIO.FXRightEncPos);
        SmartDashboard.putNumber("startBallPos", startBallPos);
    }

    public static class PeriodicIO {
        // LOGGING
        public  int    schedDeltaDesired;
        public  double schedDeltaActual;
        public  double schedDuration;
        private double lastSchedStart;

        // INPUTS
        public double FXLeftEncPos;
        public double FXRightEncPos;
        public double FXLeftCurrent;
        public double FXRightCurrent;
        public String FXLeftFaults;
        public String FXRightFaults;
        public double currentPos; //end pos
        
        //OUTPUTS
        public PTO    PTODemand;
        public double indexerDemand;
        public double climberDemand;
        public double indexerPosDemand;
    }
}