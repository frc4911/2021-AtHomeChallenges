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
    private final AnalogInput mAIBallEntered; //Port 1
    private final AnalogInput mAIBallTouchingShooter; //Port 0

    private double mFXLeftPIDPos;
    private double mFXRightPIDPos;

    private final double kIndexSpeed = 0.1; //.7
    private final double kLoadSpeed = 0.25; // .5 brian
    private final double kBackSpeed = -0.25;
    private final int beamBreakThreshold = 3;
    
    private double deltaFromEntrance = 3652.0; //Encoder ticks wanted after ball passing beam break *****************************************/
    private double startBallPos = 0.0; //Position right as the ball leaves beam break *******************************************************/
    private int numberOfBalls = 0;
    private double cobraStartTime = 0;
    private final double cobraMaxDuration = .5;
    private final double kBallOffset0and1 = 3562;
    private final double kBallOffset2 = 750;
    private final double kBallOffsetTolerance = 100;

    public enum SystemState {
        HOLDING,
        LOADING,
        CORRECTING,
        BACKING,
        INDEXING,
        COBRAING
    }

    public enum WantedState {
        HOLD,
        LOAD,
        CORRECT,
        BACK,
        INDEX,
        COBRA
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
                    case COBRAING:
                        newState = handleCobraing();
                        break;
                    case BACKING:
                        newState = handleBacking();
                        break;
                    case INDEXING:
                        newState = handleIndexing();
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
            mPeriodicIO.schedDeltaDesired = 0; // goto sleep
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
            mPeriodicIO.indexerDemand = speed + getLoadSpeed(); //kLoadSpeed;
            mPeriodicIO.schedDeltaDesired = 20; // stay awake to monitor beam break to jump to correcting
        }else if (!isBallEnteringSimple()){
            setWantedState(WantedState.CORRECT);
            startBallPos = mPeriodicIO.FXLeftEncPos;
        }

        return defaultStateTransfer();
    }

    // position ball based on passing through beam break
    private SystemState handleCorrecting() {
        if (mStateChanged) {            
            deltaFromEntrance = SmartDashboard.getNumber("delta from start",-1);
            if (deltaFromEntrance == -1){
                SmartDashboard.putNumber("delta from start", kBallOffset0and1);
                deltaFromEntrance = kBallOffset0and1;
            }
            if(numberOfBalls == 2){
                deltaFromEntrance -= kBallOffset2;
            }
            mPeriodicIO.indexerPosDemand = startBallPos + deltaFromEntrance;
            mPeriodicIO.indexerDemand = 0;
            mPeriodicIO.schedDeltaDesired = 20;
            numberOfBalls++;
            System.out.println("startBallPos - " + startBallPos);
        }
        System.out.println("IndexCorrecting target:"+mPeriodicIO.indexerPosDemand+" current pos:"+mPeriodicIO.FXLeftEncPos+" delta:"+(mPeriodicIO.indexerPosDemand-mPeriodicIO.FXLeftEncPos));
        if (Math.abs(mPeriodicIO.FXLeftEncPos - mPeriodicIO.indexerPosDemand) < kBallOffsetTolerance || isFullyLoaded()) {
            setWantedState(WantedState.HOLD);
        }

        return defaultStateTransfer();
    }

    double cobraTarget = 0;
    private SystemState handleCobraing() {
        if (mStateChanged) {
            // mPeriodicIO.indexerDemand = 0;
            mPeriodicIO.schedDeltaDesired = 20; // stay awake to monitor progress
            cobraStartTime = Timer.getFPGATimestamp();
            cobraTarget = mPeriodicIO.FXLeftEncPos-500;
            mPeriodicIO.indexerPosDemand = cobraTarget;
        }

        if (isBallEnteringSimple() || !isFullyLoaded() || Timer.getFPGATimestamp()-cobraStartTime > cobraMaxDuration){//} || mPeriodicIO.FXLeftEncPos < mPeriodicIO.indexerPosDemand+100){
            setWantedState(WantedState.HOLD);
        }
        return defaultStateTransfer();
    }

    private SystemState handleBacking() {
        if (mStateChanged) {
            mPeriodicIO.indexerDemand = kBackSpeed;
            mPeriodicIO.schedDeltaDesired = 50; // to monitor encoder pos by automode action
            numberOfBalls = 0;
        }

        return defaultStateTransfer();
    }

    double encoderCount = 0;
    double kTicksToUnload = 20000; // needs tuning
    private SystemState handleIndexing() {
        if (mStateChanged) {
            mPeriodicIO.indexerDemand = kIndexSpeed;
            mPeriodicIO.schedDeltaDesired = 0; // goto sleep
            encoderCount = mPeriodicIO.FXLeftEncPos;
        }

        if (mPeriodicIO.FXLeftEncPos > encoderCount+kTicksToUnload){
            numberOfBalls = 0;
        }
        return defaultStateTransfer();
    }

    // method needs to be called at different cycle times
    private synchronized boolean isBallEnteringSimple() {
        //return !mDIBallEntered.get();
        return mAIBallEntered.getVoltage() < beamBreakThreshold;
    }

    public synchronized boolean isBallEntering() {
        if(isBallEnteringSimple()){
            // this unorthodox code is trying to turn on the indexer asap when a ball is detected
            // getLoadSpeed changes the speed to overcome the drag of already loaded balls
            mPeriodicIO.indexerDemand = kLoadSpeed + getLoadSpeed();
            mFXLeft.set(ControlMode.PercentOutput, mPeriodicIO.indexerDemand);
            mFXRight.set(ControlMode.PercentOutput, mPeriodicIO.indexerDemand);
            return true;
        }
        return false;
    }

    public synchronized double getCurrentPosition() {
        return mPeriodicIO.FXLeftEncPos;
    }

    // method needs to be called at different cycle times
    public synchronized boolean isFullyLoaded(boolean stop) {
        if(isFullyLoaded()){
            if(stop){
                mPeriodicIO.indexerDemand = 0;
                mFXLeft.set(ControlMode.PercentOutput, mPeriodicIO.indexerDemand);
                mFXRight.set(ControlMode.PercentOutput, mPeriodicIO.indexerDemand);
            }
            return true;
        }
        return false;
    }
    public synchronized boolean isFullyLoaded() {
        return mAIBallTouchingShooter.getVoltage() < beamBreakThreshold;
    }


    // use different speeds to overcome drag of already loaded balls
    public synchronized double getLoadSpeed() {
        if(numberOfBalls == 0){
            return 0;
        }else if(numberOfBalls == 1){
            return 0.05;
        }else{
            return 0.1;
        }
    }

    public synchronized int getNumberOfBalls() {
        return numberOfBalls;
    }

    public synchronized void setNumberOfBalls(int balls) {
        numberOfBalls = balls;
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
            case COBRA:
                return SystemState.COBRAING;
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
                        isBallEnteringSimple()+","+
                        isFullyLoaded()+","+
                        mPeriodicIO.indexerDemand+","+
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
                        isBallEnteringSimple()+","+
                        isFullyLoaded()+","+
                        mPeriodicIO.indexerDemand+","+
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
        mPeriodicIO.FXLeftEncPos  = mFXLeft.getSelectedSensorPosition();
        mPeriodicIO.FXRightEncPos = mFXRight.getSelectedSensorPosition();
     }

    @Override
    public void writePeriodicOutputs() {
        if(mSystemState == SystemState.CORRECTING || mSystemState == SystemState.COBRAING){
            mFXLeft.set(ControlMode.Position, mPeriodicIO.indexerPosDemand);
            mFXRight.set(ControlMode.Position, mPeriodicIO.indexerPosDemand);
        }else{
            mFXLeft.set(ControlMode.PercentOutput, mPeriodicIO.indexerDemand);
            mFXRight.set(ControlMode.PercentOutput, mPeriodicIO.indexerDemand);
        }
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
        SmartDashboard.putBoolean("Ball Entered", isBallEnteringSimple());
        SmartDashboard.putBoolean("Fully Loaded", isFullyLoaded());
        SmartDashboard.putNumber("Enterance Beam Break Analog", mAIBallEntered.getVoltage());
        SmartDashboard.putNumber("Exit Beam Break Analog", mAIBallTouchingShooter.getVoltage());
        SmartDashboard.putNumber("Left Indexer Enc", mPeriodicIO.FXLeftEncPos);
        SmartDashboard.putNumber("Right Indexer Enc", mPeriodicIO.FXRightEncPos);
        SmartDashboard.putNumber("startBallPos", startBallPos);
        SmartDashboard.putNumber("Number of Balls", numberOfBalls);
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
        public double indexerDemand;
        public double indexerPosDemand;
    }
}