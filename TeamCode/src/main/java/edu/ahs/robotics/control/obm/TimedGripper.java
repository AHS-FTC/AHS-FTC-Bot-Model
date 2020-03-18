package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.ftc.FTCUtilities;

/**
 * Takes MotionConfig in use and changes the targetHeading at a specific location on the field.
 * Can be used at times where the heading of the robot should be changed mid movement.
 * @author Alex Appleby
 */
public class TimedGripper implements OBMCommand {

    private Ardennes ardennes;

    private State state = State.INITIAL;

    private SerialServo gripper;

    long startTime;
    long waitTime;

    private enum State{
        INITIAL,
        RUNNING,
        FINISHED
    }

    public TimedGripper(Ardennes ardennes, long waitTime) {
        this.ardennes = ardennes;
        this.waitTime = waitTime;

        gripper = ardennes.getGripper();

    }

    @Override
    public boolean check(OdometrySystem.State robotState) {
        switch(state){
            case FINISHED:
                break;

            case INITIAL:
                startTime = FTCUtilities.getCurrentTimeMillis();
                state = State.RUNNING;
                break;

            case RUNNING:
                if ((FTCUtilities.getCurrentTimeMillis() - startTime) > waitTime){
                    state = State.FINISHED;
                    gripper.setPosition(1);
                }
                break;

        }
        return false;
    }

    @Override
    public void reset() {
        state = State.INITIAL;
    }

    public void resetWaitTime(long waitTime) {this.waitTime = waitTime;}

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }
}
