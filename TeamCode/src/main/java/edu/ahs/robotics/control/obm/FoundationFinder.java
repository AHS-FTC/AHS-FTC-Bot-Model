package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.control.MotionConfig;
import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.seasonrobots.Ardennes;

/**
 * Takes MotionConfig in use and changes the targetHeading at a specific location on the field.
 * Can be used at times where the heading of the robot should be changed mid movement.
 * @author Alex Appleby
 */
public class FoundationFinder implements OBMCommand {
    private double startLookingY;

    private Ardennes ardennes;
    private TriggerDistanceSensor foundationTrigger;
    private Path path;

    private State state = State.INITIAL;

    private enum State{
        INITIAL,
        LOOKING,
        FINISHED
    }

    public FoundationFinder(Ardennes ardennes, double startLookingY, boolean redSide) {
        this.startLookingY = startLookingY;
        this.ardennes = ardennes;
        this.path = path;

        if (redSide) {
            this.foundationTrigger = ardennes.getFoundationTriggerRight();
        } else {
            this.foundationTrigger = ardennes.getFoundationTriggerLeft();
        }
    }

    @Override
    public boolean check(OdometrySystem.State robotState) {
        switch(state){
            case FINISHED:
                return true;
            case INITIAL:
                if (robotState.position.y >= startLookingY){
                    state = State.LOOKING;
                }
                break;
            case LOOKING:
                if (foundationTrigger.isTriggered()){
                    state = State.FINISHED;
                    return true;
                }
                break;
        }
        return false;
    }

    @Override
    public void reset() {
        state = State.INITIAL;
    }

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }
}
