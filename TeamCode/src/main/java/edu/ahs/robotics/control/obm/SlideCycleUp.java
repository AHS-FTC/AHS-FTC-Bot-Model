package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.Slides;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.ftc.FTCUtilities;

public class SlideCycleUp implements OBMCommand{

    private int cycleHeight;
    private static final double STATIC_POWER = 0.2;
    private static final double UP_POWER = 0.9;
    private Ardennes ardennes;
    private State state;

    private Slides slides;
    private SerialServo xSlide;
    private SerialServo gripper;

    private long startTime = 0L;

    private enum State{
        INITIAL,
        RAISINGZ,
        FINISHED
    }

    public SlideCycleUp(Ardennes ardennes){
        this(ardennes, 280);
    }

    public SlideCycleUp(Ardennes ardennes, int cycleHeight){
        this.ardennes = ardennes;
        state  = State.INITIAL;

        slides = ardennes.getSlides();
        xSlide = ardennes.getxSlide();
        gripper = ardennes.getGripper();

        this.cycleHeight = cycleHeight;
    }

    public void setCycleHeight(int cycleHeight) {
        this.cycleHeight = cycleHeight;
    }

    @Override
    public boolean check(OdometrySystem.State robotState){
        switch (state){
            case FINISHED:
                break;

            case INITIAL:
                if (robotState.position.y > -12) { // if we're 12 inches above the center of the field
                    state = State.RAISINGZ;
                    slides.runAtPower(UP_POWER);
                }
                break;

            case RAISINGZ:
                if(slides.getCurrentPosition() > cycleHeight){
                    state = State.FINISHED;
                    slides.runAtPower(STATIC_POWER);
                    xSlide.setPosition(1);
                    startTime = FTCUtilities.getCurrentTimeMillis();
                }
                break;
        }
        return false;
    }

    @Override
    public void reset() {
        state = State.INITIAL;
    }

    public boolean isFinished(){
        return state == State.FINISHED;
    }
}
