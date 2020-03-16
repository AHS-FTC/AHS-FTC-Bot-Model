package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.hardware.sensors.DistanceSensor;
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

    private State state;

    private enum State{
        WAITING,
        FINISHED
    }

    public BlockGripper(Ardennes ardennes){
        this.ardennes = ardennes;
        this.gripperTrigger = ardennes.getGripperTrigger();
        reset();
    }

    @Override
    public boolean check(OdometrySystem.State robotState) {
        FTCUtilities.addData("isTriggered", gripperTrigger.isTriggered());
        FTCUtilities.addData("distance", ((DistanceSensor)gripperTrigger).getDistOptimized());
        FTCUtilities.updateOpLogger();
        switch(state){
            case FINISHED:
                return true;
            case WAITING:
                if(gripperTrigger.isTriggered()) {// || FTCUtilities.getCurrentTimeMillis() - startTime > waitTime){ gripperTrigger.isTriggered()
                    ardennes.getGripper().setPosition(1);
                    ardennes.getIntake().stopMotors();
                    state = State.FINISHED;
                    return true;
                }
                break;
        }
        return false;
    }

    public void reset(){
        state = State.WAITING;
    }

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }
}
