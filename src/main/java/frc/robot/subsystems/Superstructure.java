package frc.robot.subsystems;

import java.util.Optional;

import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.auto.SmartDashboardInteractions;
import frc.robot.subsystems.Limelights.ShootwardsLimelight;

public class Superstructure extends Subsystem {

    private Swerve    mSwerve    = null;
    private Indexer   mIndexer   = null;
    private Collector mCollector = null;
    //private Donger    mDonger    = null;
    private Shooter   mShooter   = null;
    private ShootwardsLimelight mShootwardsLimelight = null;
    private RobotState mRobotState = null;
    // private CollectwardsLimelight mCollectwardsLimelight = null;

    public enum SystemState {
        HOLDING,
        COLLECTING,
        SHOOTING,
        CLEARING_BALLS,
        MANUAL_SHOOTING
    }

    public enum WantedState {
        HOLD,
        COLLECT,
        SHOOT,
        CLEAR_BALLS,
        MANUAL_SHOOT
    }

    private SystemState   mSystemState = SystemState.HOLDING;
    private WantedState   mWantedState = WantedState.HOLD;
    private boolean       mStateChanged;
    private boolean       mShootSetup = true;
    private double        mDistance;
    private double        mRPM;
    private final boolean mLoggingEnabled = true;  // used to disable logging for this subsystem only
    public PeriodicIO     mPeriodicIO;
    private double        mLastDistanceToGoal;
    @SuppressWarnings("unused")
    private int           mListIndex;
    private Optional<AimingParameters> mAimingParameters;
    private int           mFastCycle = 10;
    private int           mSlowCycle = 100;
    private double           timeout = 0;

    private static String sClassName;
    private static int sInstanceCount;
    private static Superstructure sInstance = null;
    public  static Superstructure getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new Superstructure(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+"getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Superstructure(String caller) {
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mSwerve              = Swerve.getInstance(sClassName);
        mIndexer             = Indexer.getInstance(sClassName);
        mCollector           = Collector.getInstance(sClassName);
        mShooter             = Shooter.getInstance(sClassName);
        //mDonger              = Donger.getInstance(sClassName);
        mShootwardsLimelight = ShootwardsLimelight.getInstance(sClassName);
        mRobotState          = RobotState.getInstance(sClassName);
        // mCollectwardsLimelight = CollectwardsLimelight.getInstance(sClassName);

        mPeriodicIO = new PeriodicIO();
    }

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(Phase phase) {
            synchronized(Superstructure.this) {
                mSystemState = SystemState.HOLDING;
                mWantedState = WantedState.HOLD;
                mStateChanged = true;
                System.out.println(sClassName + " onStart state " + mSystemState);
                switch (phase) {
                    case DISABLED:
                        mPeriodicIO.schedDeltaDesired = 0; // goto sleep
                        break;
                    default:
                        mPeriodicIO.schedDeltaDesired = 100;
                        break;
                }
                stop();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(Superstructure.this) {
                SystemState newState;
                switch(mSystemState) {
                    case COLLECTING:
                        newState = handleCollecting();
                        break;
                    case SHOOTING:
                        newState = handleShooting();
                        break;
                    case CLEARING_BALLS:
                        newState = handleClearingBalls();
                        break;
                    case MANUAL_SHOOTING:
                        newState = handleManualShooting();
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
            mIndexer.setWantedState(Indexer.WantedState.HOLD, sClassName);
            mCollector.setWantedState(Collector.WantedState.HOLD, sClassName);
            mShooter.setWantedState(Shooter.WantedState.HOLD, sClassName);
            
            mShootwardsLimelight.setWantedState(ShootwardsLimelight.WantedState.TARGETLED);//brian
            mPeriodicIO.schedDeltaDesired = mSlowCycle;

            // timeout = SmartDashboard.getNumber("manual shoot timeout (msec)",-1);
            // if (timeout == -1){
            //     SmartDashboard.putNumber("manual shoot timeout (msec)",150);
            //     timeout = 150;
            // }
        }

        return defaultStateTransfer();
    }

    private SystemState handleCollecting() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mFastCycle;
        }

        if (!mIndexer.isFullyLoaded(true)) 
        {
            mCollector.setWantedState(Collector.WantedState.COLLECT, sClassName);
            if (mIndexer.getWantedState() == Indexer.WantedState.HOLD) {
                if (mIndexer.isBallEntering()) {
                    if (mIndexer.getNumberOfBalls() != 2){
                        mIndexer.setWantedState(Indexer.WantedState.LOAD, sClassName);
                    }
                    else{
                        mIndexer.setWantedState(Indexer.WantedState.HOLD, sClassName);
                        mCollector.setWantedState(Collector.WantedState.HOLD, sClassName);
                    }
                }
            }
        } else {
            mIndexer.setWantedState(Indexer.WantedState.HOLD, sClassName); //COBRA
            mCollector.setWantedState(Collector.WantedState.HOLD, sClassName);
        }
        
        return collectingStateTransfer();
    }
    
    private SystemState handleShooting() {
        if (mStateChanged) {
            // mShootwardsLimelight.setWantedState(ShootwardsLimelight.WantedState.TARGETLED);
            mSwerve.limeLightAim();
            mLastDistanceToGoal = Double.MIN_VALUE;
            mPeriodicIO.schedDeltaDesired = mFastCycle;
        }
        System.out.println("SuperStructure handle shooting");
        //mShooter.setShootRPM(4000); 
        // double distanceToGoal = getDistanceToGoal();
        // if (mLastDistanceToGoal != distanceToGoal){
        //    mShooter.setShootRPM(distanceToGoal); // TODO: check if only need to call once
            // mLastDistanceToGoal = distanceToGoal;
        // }

        if (readyToShootAndOnTarget()) {
            mIndexer.setWantedState(Indexer.WantedState.INDEX, sClassName);
            // if (mIndexer.isBallEntering()) {
            //     mDonger.setWantedState(Donger.WantedState.COLLECT);
            // } else {
            //     mDonger.setWantedState(Donger.WantedState.HOLD);
            // }

            mShootSetup = false;
        } else {
            mIndexer.setWantedState(Indexer.WantedState.HOLD, sClassName);
            // if (mIndexer.isFullyLoaded()) {
            //     mDonger.setWantedState(Donger.WantedState.SECURE);
            // } else {
            //     mDonger.setWantedState(Donger.WantedState.HOLD);
            // }
        }

        return shootingStateTransfer();
    }

    private SystemState handleClearingBalls() {
        if (mStateChanged) {
            mPeriodicIO.schedDeltaDesired = mSlowCycle;
            mCollector.setWantedState(Collector.WantedState.BACK, sClassName);
            mIndexer.setWantedState(Indexer.WantedState.BACK, sClassName); //Matthew
        }

        // Matthew changed from collectingStateTransfer to defaultStateTransfer
        // brian changed to clearingStateTransfer to stop indexer
        return clearingStateTransfer();
    }

    int count = 0;
    private SystemState handleManualShooting2() {
        if (mStateChanged) {
            count = 0;
            mSwerve.limeLightAim();
            if(mDistance>0){
                System.out.println("~~~~~~~~~TRY TO SET SHOOT DISTANCE~~~~~~~~");
                //mShooter.setShootDistance(mDistance);
            }
            else{
                System.out.println(sClassName+", set shoot RPM: "+mRPM);
                mShooter.setShootRPM(mRPM);
            }
        }

        mPeriodicIO.schedDeltaDesired = mFastCycle;
        if (readyToShootAndOnTarget() || !mShootSetup) {
        // if (mShooter.readyToShoot() || !mShootSetup) {

            if (++count>timeout){
                mIndexer.setWantedState(Indexer.WantedState.INDEX, sClassName);
                mCollector.setWantedState(Collector.WantedState.COLLECT, sClassName);

            }
            
            mShootSetup = false;
        }
        else{
            mIndexer.setWantedState(Indexer.WantedState.HOLD, sClassName);
        }

        return shootingStateTransfer();
    }

    private SystemState handleManualShooting() {
        if (mStateChanged) {
            mShooter.setShootRPM(mRPM);
        }

        mPeriodicIO.schedDeltaDesired = mFastCycle;
        mIndexer.setWantedState(Indexer.WantedState.INDEX, sClassName);
        mCollector.setWantedState(Collector.WantedState.COLLECT, sClassName);
        return shootingStateTransfer();
    }

    private SystemState collectingStateTransfer() {
        if (mWantedState != WantedState.COLLECT) {
            mCollector.setWantedState(Collector.WantedState.HOLD, sClassName);
            mIndexer.setWantedState(Indexer.WantedState.HOLD, sClassName);
        }

        return defaultStateTransfer();
    }

    private SystemState clearingStateTransfer() {
        if (mWantedState != WantedState.CLEAR_BALLS) {
            mCollector.setWantedState(Collector.WantedState.HOLD, sClassName);
            mIndexer.setWantedState(Indexer.WantedState.HOLD, sClassName);
        }

        return defaultStateTransfer();
    }

    private SystemState shootingStateTransfer() {
        if (mWantedState != WantedState.SHOOT && mWantedState != WantedState.MANUAL_SHOOT) {
            mShootSetup = true;
            mShooter.setWantedState(Shooter.WantedState.HOLD, sClassName);
            mIndexer.setWantedState(Indexer.WantedState.HOLD, sClassName);
            mCollector.setWantedState(Collector.WantedState.HOLD, sClassName);
                        
            // setShooterHoldSpeed(0.45);
        }

        return defaultStateTransfer();
    }

    // public synchronized void setManualShootDistance(double distance) {
    //     mDistance = distance;
    //     mRPM = 0;
    //     setWantedState(WantedState.MANUAL_SHOOT, sClassName);
    // }

    public synchronized void setManualShootRPM(double rpm) {
        mRPM = rpm;
        mDistance = 0;
        if (mWantedState!= WantedState.MANUAL_SHOOT){
            setWantedState(WantedState.MANUAL_SHOOT, sClassName);
        }
    }

    public synchronized void setShooterHoldSpeed(double speed) {
        mShooter.setHoldRPM(speed);
    }

    private boolean readyToShootAndOnTarget() {
        return mSwerve.isOnTarget();
        // return (mShooter.readyToShoot() && mSwerve.isOnTarget()) || !mShootSetup;
    }

    private double getDistanceToGoal() {
        mAimingParameters = mRobotState.getOuterGoalParameters();
        double distance = 0.0;
        if (mAimingParameters.isPresent()) {
            distance = mAimingParameters.get().getRobotToGoal().getTranslation().norm() / 12.0;
        }

        return distance;
    }

    public synchronized WantedState getWantedState() {
        return mWantedState;
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case COLLECT:
                return SystemState.COLLECTING;
            case SHOOT:
                return SystemState.SHOOTING;
            case CLEAR_BALLS:
                return SystemState.CLEARING_BALLS;
            case MANUAL_SHOOT:
                return SystemState.MANUAL_SHOOTING;
            case HOLD:
                return SystemState.HOLDING;
        }
        return null;
    }

    public synchronized void setWantedState(WantedState state, String caller) {
        if (mWantedState != state){
            mPeriodicIO.schedDeltaDesired = 2;
        }
        System.out.println(sClassName+" setWantedState " + state + " ("+caller+")");
        mWantedState = state;
    }

    @Override
    public void stop() {
       
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mListIndex = enabledLooper.register(mLoop);
    }

    @Override
    public String getLogHeaders() {
        if (mLoggingEnabled){
            return sClassName+".systemState,"+
                   sClassName+".shootSetup,"+
                   sClassName+".readyToShootAndOnTarget,"+
                   sClassName+".schedDeltaDesired,"+
                   sClassName+".schedDeltaActual,"+
                   sClassName+".schedDuration";
        }
        return null;
    }

    private String generateLogValues(boolean telemetry){
        String values;
        if (telemetry) {
            values = ""+mSystemState+","+
                        mShootSetup+","+
                        mPeriodicIO.readyToShootAndOnTarget+","+
                        /*mPeriodicIO.schedDeltaDesired+*/","+
                        /*mPeriodicIO.schedDeltaActual+*/","
                        /*mPeriodicIO.schedDuration*/;

        }
        else {
            mPeriodicIO.schedDuration = Timer.getFPGATimestamp() - mPeriodicIO.lastSchedStart;
            values = ""+mSystemState+","+
                        mShootSetup+","+
                        mPeriodicIO.readyToShootAndOnTarget+","+
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
        mPeriodicIO.readyToShootAndOnTarget = readyToShootAndOnTarget();
    }

    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public int whenRunAgain () {
        if (mStateChanged && mPeriodicIO.schedDeltaDesired==0){
            return 1; // one more loop before going to sleep
        }
        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Ready To Shoot and On Target", mPeriodicIO.readyToShootAndOnTarget);
        SmartDashboard.putBoolean("Ready To Shoot", mShooter.readyToShoot());
        SmartDashboard.putBoolean("On Target", mSwerve.isOnTarget());
        SmartDashboard.putBoolean("Shoot Setup", mShootSetup);
    }

    public static class PeriodicIO {
        // LOGGING
        public int schedDeltaDesired;
        public double schedDeltaActual;
        public double schedDuration;
        private double lastSchedStart;

        // INPUTS
        public boolean readyToShootAndOnTarget;

        //OUTPUTS
    }
}
