package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.Slides;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;

public class SlideCycle implements OBMCommand{

    private static final int CYCLE_HEIGHT = 200;
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
        this.ardennes = ardennes;
        state  = State.INITIAL;

        slides = ardennes.getSlides();
        xSlide = ardennes.getySlide();
        gripper = ardennes.getGripper();
    }

    @Override
    public void check(OdometrySystem.State robotState){
        switch (state){
            case FINISHED:
                break;

            case INITIAL:
                if (robotState.position.y > 12) { // if we're 12 inches above the center of the field
                    state = State.RAISINGZ;
                    slides.runAtPower(UP_POWER);
                }
                break;
            case RAISINGZ:
                if(slides.getCurrentPosition() > CYCLE_HEIGHT){
                    state = State.EXTENDINGX;
                    slides.runAtPower(STATIC_POWER);
                    xSlide.setPosition(1);
                    startTime = FTCUtilities.getCurrentTimeMillis();
                }
                break;
            case EXTENDINGX:
                if(FTCUtilities.getCurrentTimeMillis() - startTime > 1000){
                    state = State.DROPPINGBLOCK;
                    gripper.setPosition(0);
                    startTime = FTCUtilities.getCurrentTimeMillis();
                }
                break;
            case DROPPINGBLOCK:
                if(FTCUtilities.getCurrentTimeMillis() - startTime > 250){
                    state = State.RETRACTINGX;
                    xSlide.setPosition(0);
                    startTime = FTCUtilities.getCurrentTimeMillis();
                }
                break;
            case RETRACTINGX:
                if(FTCUtilities.getCurrentTimeMillis() - startTime > 1000){
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
    }
}