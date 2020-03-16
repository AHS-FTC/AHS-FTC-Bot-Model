package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.ftc.FTCUtilities;

/**
 * Takes MotionConfig in use and changes the targetHeading at a specific location on the field.
 * Can be used at times where the heading of the robot should be changed mid movement.
 * @author Alex Appleby
 */
public class FoundationGrip implements OBMCommand {
    private double startLookingY;

    private Ardennes ardennes;

    private State state = State.INITIAL;

    private SerialServo foundationLeft;
    private SerialServo foundationRight;

    long startTime;
    long waitTime;

    private enum State{
        INITIAL,
        GRABBING,
        FINISHED
    }

    public FoundationGrip(Ardennes ardennes, long waitTime) {
        this.ardennes = ardennes;
        this.waitTime = waitTime;

        foundationLeft = ardennes.getLeftFoundation();
        foundationRight = ardennes.getRightFoundation();

        startTime = FTCUtilities.getCurrentTimeMillis();
    }

    @Override
    public boolean check(OdometrySystem.State robotState) {
        switch(state){
            case FINISHED:
                break;

            case INITIAL:
                if (FTCUtilities.getCurrentTimeMillis() - startTime > waitTime){
                    state = State.GRABBING;
                    foundationRight.setPosition(1);
                    foundationLeft.setPosition(1);
                }
                break;

            case GRABBING:
                if (FTCUtilities.getCurrentTimeMillis() - startTime > 400){
                    state = State.FINISHED;
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
