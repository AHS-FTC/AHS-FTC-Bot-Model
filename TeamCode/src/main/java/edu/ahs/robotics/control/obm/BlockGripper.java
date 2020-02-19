package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.hardware.sensors.Trigger;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.ftc.FTCUtilities;

/**
 * OBMCommand for gripping the blocks with delay or upon triggering of distanceSensor.
 * @author Alex Appleby
 */
public class BlockGripper implements OBMCommand {

    private Ardennes ardennes;

    private Trigger gripperTrigger;

    private long startTime;
    private long waitTime;

    private State state;

    private enum State{
        INITIAL,
        WAITING,
        FINISHED
    }

    public BlockGripper(Ardennes ardennes, long waitTime){
        this.ardennes = ardennes;
        this.waitTime = waitTime;
        this.gripperTrigger = ardennes.getGripperTrigger();
        reset();
    }

    public void resetWaitTime(long waitTime){
        this.waitTime = waitTime;
        startTime = FTCUtilities.getCurrentTimeMillis();
    }

    @Override
    public boolean check(OdometrySystem.State robotState) {
        switch(state){
            case FINISHED:
                break;
            case INITIAL:
                startTime = FTCUtilities.getCurrentTimeMillis();
                state = State.WAITING;
                break;
            case WAITING:
                if(FTCUtilities.getCurrentTimeMillis() - startTime > waitTime) {// || FTCUtilities.getCurrentTimeMillis() - startTime > waitTime){ gripperTrigger.isTriggered()
                    ardennes.getGripper().setPosition(1);
                    ardennes.getIntake().stopMotors();
                    state = State.FINISHED;
                }
                break;
        }
        return false;
    }

    public void reset(){
        state = State.INITIAL;
    }

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }
}
