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
 * The main things this subsystem has to do are intake balls, unjam itself, and lower and raise the mechanism. 
 * 
 * @see Subsystem.java
 */
public class Intake extends Subsystem {
    
   

    private static Intake sInstance = null; //Create null instance

    public static Intake getInstance() { //Initalize instance
        if (sInstance == null) {
            sInstance = new Intake();
        }
        return sInstance;
    }

    private final NidecMotor mRoller;
    private final IRSensor mIRAcross;
    
    public Intake() { //Initialize part constants
        mRoller=new NidecMotor(Constants.kIntakeRollerPort);
        mRoller.changeControlMode(NidecControlMode.Only_PWM);
       
        mIRAcross = new IRSensor(Constants.kIntakeIRHopperPort, Constants.kIntakeRollerMin, Constants.kIntakeRollerMax);
          
    }
    
    //SystemState defines the possible states for the intake, and is set to the wanted state when a change is needed
    public enum SystemState {
        ACCUMULATING, //feed a ball at a time
        UNJAMMING, //Reverses motors continuously
        IDLE, // stop all motors
    }   //lowering and raising just a separate thing

    public enum WantedState { //Possible states that SystemState can be set to
        IDLE,
        UNJAM,
        INTAKE
    }
    
    //Current state is idle, desired state is idle
    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;
    
    private double mCurrentStateStartTime; //time isnce current ste was switched to
    private boolean mStateChanged; //true if state is changed

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            stop();
            synchronized (Intake.this) { //Make all the following code excecute in sequence. equivalent to C#'s Lock() block 
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
                mCurrentStateStartTime = timestamp;
            } //Disenggae thread-lock
        }

        @Override
        public void onLoop(double timestamp) {
             
            hopperSense(timestamp); //Check if hopper is full
            
            synchronized (Intake.this) { //Engage Threadlock
                SystemState newState;
                switch (mSystemState) { //Oh boy here we go
                case IDLE: //Switch to IDLE
                    newState = handleIdle();
                    break;
                case UNJAMMING: //Switch to UNJAMMING and give it the timestamp and curnt state duration
                    newState = handleUnjamming(timestamp, mCurrentStateStartTime);
                    break;                
                case ACCUMULATING: //Switch to ACCUMULATING
                    newState = handleAccumulating();
                    break;                 
                default: //IDLE is deafult
                    newState = SystemState.IDLE;
                }
                if (newState != mSystemState) { //If new state is dfferent from current state
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
        public void onStop(double timestamp) { //stop the robot?
            stop();
        }
    };

    private double firstSenseStamp=0;
    private boolean counting=false; //Is the hopper counting
    private boolean hopperFull=false; //Is the hopper full
    private void hopperSense(double now) { //Detect level
        if(mIRAcross.seesBall()) { //If a ball is detected                          
            if(counting) { //if counting is true
                if (now - firstSenseStamp > Constants.kHopperSensePeriod) { //If it isnt a bounce
                    hopperFull=true; //Hopper is full                  
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
    
    /* SWITCH STATEMENT EXPLANATION
    switch (variable or expression) - checks the input to see if it is equal to the value of any case. If no case matches, it will check the default case instead
    {
        case 1: - if argument = 1, the code in this case runs
        CaseIs1();
        break;
        
        case 2:
        CaseIs2();
        break;
    }
    */
    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
        case INTAKE: //If mWantedState == INTAKE
            return SystemState.ACCUMULATING;
        case UNJAM:
            return SystemState.UNJAMMING;
        default:
            return SystemState.IDLE;
        }
    }

    private SystemState handleIdle() { //If state is idle
        mRoller.set(0); //Shut down rollers
        return defaultStateTransfer(); //Call switch above
    }

    private SystemState handleUnjamming(double now, double startStartedAt) { //Unjam method
        mRoller.set(Constants.kIntakeUnjamPower); //Set roller speed to unjam speed as specified elsewhere
        SystemState newState = SystemState.UNJAMMING;
        if (now - startStartedAt > Constants.kIntakeUnjamPeriod) { /*if the current time minus the time the subsytem has been 
running is greater than  the specified unjam time*/
            newState = SystemState.IDLE; //set tate to idle
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
