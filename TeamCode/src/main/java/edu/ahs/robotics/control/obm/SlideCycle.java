package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.Slides;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.ftc.FTCUtilities;

public class SlideCycle implements OBMCommand{

    private int cycleHeight;
    private static final double STATIC_POWER = 0.2;
    private static final double UP_POWER = 0.6;
    private Ardennes ardennes;
    private State state;

    private Slides slides;
    private SerialServo xSlide;
    private SerialServo gripper;

    private long startTime = 0L;

    private enum State{
        INITIAL,
        RAISINGZ,
        EXTENDINGX,
        DROPPINGBLOCK,
        RETRACTINGX,
        LOWERINGZ,
        FINISHED
    }

    public SlideCycle(Ardennes ardennes){
        this(ardennes, 160);
    }

    public SlideCycle(Ardennes ardennes, int cycleHeight){
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
                if (robotState.position.y > 6) { // if we're 12 inches above the center of the field
                    state = State.RAISINGZ;
                    slides.runAtPower(UP_POWER);
                }
                break;
            case RAISINGZ:
                if(slides.getCurrentPosition() > cycleHeight){
                    state = State.EXTENDINGX;
                    slides.runAtPower(STATIC_POWER);
                    xSlide.setPosition(1);
                    startTime = FTCUtilities.getCurrentTimeMillis();
                }
                break;
            case EXTENDINGX:
                if(FTCUtilities.getCurrentTimeMillis() - startTime > 600){
                    state = State.DROPPINGBLOCK;
                    gripper.setPosition(0);
                    startTime = FTCUtilities.getCurrentTimeMillis();
                }
                break;
            case DROPPINGBLOCK:
                if(FTCUtilities.getCurrentTimeMillis() - startTime > 50){
                    state = State.RETRACTINGX;
                    xSlide.setPosition(0);
                    startTime = FTCUtilities.getCurrentTimeMillis();
                }
                break;
            case RETRACTINGX:
                if(FTCUtilities.getCurrentTimeMillis() - startTime > 200){
                    state = State.LOWERINGZ;
                    slides.runAtPower(-.2);
                }
                break;
            case LOWERINGZ:
                if(slides.atBottom()){
                    slides.stopMotors();
                    state = State.FINISHED;
                }
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
