package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.hardware.sensors.Trigger;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;

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

    @Override
    public void check(OdometrySystem.State robotState) {
        switch(state){
            case FINISHED:
                break;
            case INITIAL:
                startTime = FTCUtilities.getCurrentTimeMillis();
                state = State.WAITING;
                break;
            case WAITING:
                if(gripperTrigger.isTriggered() || FTCUtilities.getCurrentTimeMillis() - startTime > waitTime){
                    ardennes.getGripper().setPosition(1);
                    ardennes.getIntake().stopMotors();
                    state = State.FINISHED;
                }
                break;
        }
    }

    public void reset(){
        state = State.INITIAL;
    }

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }
}
