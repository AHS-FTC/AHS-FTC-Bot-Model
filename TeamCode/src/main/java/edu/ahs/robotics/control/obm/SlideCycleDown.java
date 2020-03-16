package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.Slides;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.ftc.FTCUtilities;

public class SlideCycleDown implements OBMCommand{

    private Ardennes ardennes;
    private State state;

    private Slides slides;
    private SerialServo xSlide;
    private SerialServo gripper;

    private long startTime = 0L;

    private enum State{
        INITIAL,
        RETRACTINGX,
        LOWERINGZ,
        FINISHED
    }

    public SlideCycleDown(Ardennes ardennes){
        this.ardennes = ardennes;
        state  = State.INITIAL;

        slides = ardennes.getSlides();
        xSlide = ardennes.getxSlide();
        gripper = ardennes.getGripper();

    }

    @Override
    public boolean check(OdometrySystem.State robotState){
        switch (state){
            case FINISHED:
                break;

            case INITIAL:
                if(FTCUtilities.getCurrentTimeMillis() - startTime > 50){
                state = State.RETRACTINGX;
                xSlide.setPosition(0);
                startTime = FTCUtilities.getCurrentTimeMillis();
                }
                break;

            case RETRACTINGX:
                if(FTCUtilities.getCurrentTimeMillis() - startTime > 200){
                    state = State.LOWERINGZ;
                    slides.runAtPower(-.4);
                }
                break;

            case LOWERINGZ:
                if(slides.atBottom()){
                    slides.stopMotors();
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

    public boolean isFinished(){
        return state == State.FINISHED;
    }
}
