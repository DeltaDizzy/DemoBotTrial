package com.team254.frc2017.subsystems;

import edu.wpi.first.wpilibj.Timer;
import com.team254.frc2017.Constants;
import com.team254.frc2017.loops.Loop;
import com.team254.frc2017.loops.Looper;
import com.team254.lib.util.Util;
import com.team254.lib.util.drivers.IRSensor;
import com.team254.lib.util.drivers.NidecMotor;
import com.team254.lib.util.drivers.NidecMotor.NidecControlMode;

import java.util.Arrays;

/**
 * The feeder subsystem consists of a 3.3 Nidec Brushless motor used to intake balls from the ground
 * There are ir sensors placed across the top of the hopper to sense if the hopper is full.
 * The mechanism is also lowered by 2 servo linear actuators.
 * The main things this subsystem has to are intake fuel, unjam, and lower and raise the mechanism. 
 * 
 * @see Subsystem.java
 */
public class Intake extends Subsystem {
    
   

    private static Intake sInstance = null;

    public static Intake getInstance() {
        if (sInstance == null) {
            sInstance = new Intake();
        }
        return sInstance;
    }

    private final NidecMotor mRoller;
    private final IRSensor mIRAcross;
    
    public Intake() {
        mRoller=new NidecMotor(Constants.kIntakeRollerPort);
        mRoller.changeControlMode(NidecControlMode.Only_PWM);
       
        mIRAcross = new IRSensor(Constants.kIntakeIRHopperPort, Constants.kIntakeRollerMin, Constants.kIntakeRollerMax);
          
    }

    public enum SystemState {
        ACCUMULATING, //feed a ball at a time
        UNJAMMING, //Reverses motors continuously
        IDLE, // stop all motors
    }   //lowering and raising just a separate thing

    public enum WantedState {
        IDLE,
        UNJAM,
        INTAKE
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;

    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Intake.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
            }
        }

        @Override
        public void onLoop(double timestamp) {
             
            hopperSense(timestamp);
            
            synchronized (Intake.this) {
                SystemState newState;
                switch (mSystemState) {
                case IDLE:
                    newState = handleIdle();
                    break;
                case UNJAMMING:
                    newState = handleUnjamming(timestamp, mCurrentStateStartTime);
                    break;                
                case ACCUMULATING:
                    newState = handleAccumulating();
                    break;                 
                default:
                    newState = SystemState.IDLE;
                }
                if (newState != mSystemState) {
                    System.out.println("Intake state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = timestamp;
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

    private double firstSenseStamp=0;
    private boolean counting=false;
    private boolean hopperFull=false;
    private void hopperSense(double now) {
        if(mIRAcross.seesBall()) {                          
            if(counting) {
                if (now - firstSenseStamp > Constants.kHopperSensePeriod) {
                    hopperFull=true;                  
                }
            }else {
                firstSenseStamp=now;
                counting=true;
            }            
        }else {
            counting=false;
            hopperFull=false;         
        }
    }
    
    public boolean getHopperFull() {
        return hopperFull;
    }
    
    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
        case INTAKE:
            return SystemState.ACCUMULATING;
        case UNJAM:
            return SystemState.UNJAMMING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleIdle() {
        mRoller.set(0);
        return defaultStateTransfer();
    }

    private SystemState handleUnjamming(double now, double startStartedAt) {
        mRoller.set(Constants.kIntakeUnjamPower);
        SystemState newState = SystemState.UNJAMMING;
        if (now - startStartedAt > Constants.kIntakeUnjamPeriod) {
            newState = SystemState.IDLE;
        }
        switch (mWantedState) {
        case INTAKE:
            return SystemState.ACCUMULATING;
        case UNJAM:
            return newState;
        default:
            return SystemState.IDLE;
        }
    }
    

    private SystemState handleAccumulating() {
        if(hopperFull) {
            mRoller.set(0);
            System.out.println("Hopper Full");
        }else {
            mRoller.set(Constants.kIntakePower);
        }        
        return defaultStateTransfer();
    }

    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }


    @Override
    public void outputToSmartDashboard() {
        // SmartDashboard.putNumber("feeder_speed", mMasterTalon.get() / Constants.kFeederSensorGearReduction);
    }

    @Override
    public void stop() {
        setWantedState(WantedState.IDLE);
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    public boolean checkSystem() {
        System.out.println("Testing INTAKE.-----------------------------------");
        boolean failure=false;       
        return !failure;
    }

}