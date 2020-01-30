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
    private boolean canRun = true;

    public SlideCycle(Ardennes ardennes){
        this.ardennes = ardennes;
    }

    @Override
    public void check(OdometrySystem.State state) {
        if (state.position.y > 12 && canRun) { // if we're 5 inches above the center of the field
            cycleSlides();
            canRun = false;
        }
    }

    public void reset(){
        canRun = true;
    }

    public void cycleSlides(){
        Thread thread = new CycleThread();
        thread.start();
    }

    private class CycleThread extends Thread{
        @Override
        public void run() {
            Slides slides = ardennes.getSlides();
            SerialServo xSlide = ardennes.getySlide();
            SerialServo gripper = ardennes.getGripper();


            slides.runAtPower(UP_POWER);

            while (slides.getCurrentPosition() < CYCLE_HEIGHT){
                FTCUtilities.sleep(50);
            }

            slides.runAtPower(STATIC_POWER);
            xSlide.setPosition(1);
            FTCUtilities.sleep(1000);
            gripper.setPosition(0);
            FTCUtilities.sleep(250);
            xSlide.setPosition(0);
            FTCUtilities.sleep(1000);
            slides.goToBottom();        }
    }
}
