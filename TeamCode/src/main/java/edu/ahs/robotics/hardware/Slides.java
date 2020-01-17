package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import edu.ahs.robotics.control.pid.PID;
import edu.ahs.robotics.hardware.sensors.LimitSwitch;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.ParameterLookup;

/**
 * Vertical slides system for robot. Encapsulates motors and limit switches
 */
public class Slides {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private LimitSwitch limitSwitch;
    private LimitSwitch limitSwitch2;

    private double motorPower = 0;
    private int targetLevel = 0;

    private static final int ENCODER_TICKS_PER_LEVEL = 420;
    private static final int PID_ERROR_THRESHOLD = 4000;
    private static final int SLIDES_MAX = 4000; // maximum encoder val of slides
    private static final int MAX_LEVEL = 10;
    private static final double UP_POWER = 1;
    private static final double DOWN_POWER = -.1;

    private static final double P = 0.000008;// = 0.0005;
    private static final double I = 0.0;
    private static final double D = 0.0;

    private boolean frozen = false;
    private int freezeHeight = 0;

    private PID pid = new PID(P,I,D,3);

    private SlidesThread thread = new SlidesThread();

    public Slides (){
        leftMotor = FTCUtilities.getMotor("slideL");
        rightMotor = FTCUtilities.getMotor("slideR");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        thread.start();

        ParameterLookup lookup = FTCUtilities.getParameterLookup();

        limitSwitch = new LimitSwitch("limitSwitch");
        limitSwitch2 = new LimitSwitch("limitSwitch2");
    }


    /**
     * Direct control over slide motors at a specified power
     * Ensures that motors do not drive past limits.
     * @param slidesPower Value between -1 and 1 to move slides
     */
    public void runAtPower(double slidesPower) {
        if (getCurrentPosition() >= SLIDES_MAX) {
            slidesPower = Range.clip(slidesPower, -1, 0);
        } else if (atBottom()) {
            slidesPower = Range.clip(slidesPower, 0, 1);
            resetEncoders();
            targetLevel = 0;
        }

        setPower(slidesPower);
    }

    /**
     * @return true if slides are at bottom
     */
    public boolean atBottom(){
        if (limitSwitch.isTriggered() || limitSwitch2.isTriggered()) {
            return true;
        } else {
            return false;
        }
    }



    public void resetEncoders(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

//    public void startAutoControl(){
//        synchronized (thread){
//            thread.running = true;
//            thread.notifyAll();
//        }
//    }
//
//    public void stopAutoControl(){
//        synchronized (thread) {
//            frozen = false;
//            thread.running = false;
//        }
//    }

//    public void killThread(){
//        thread.kill();
//    }
//
//    public boolean isAutoControlled(){
//        return thread.running;
//    }

//    private int getTargetHeight(){
//        if(frozen){
//           return freezeHeight;
//        } else {
//            return targetLevel * ENCODER_TICKS_PER_LEVEL;
//        }
//    }

    /**
     * Sets the target level that you want the slides to achieve.
     * @param level Number of blocks high you wish to raise slides to.
     */
    public void setTargetLevel (int level) {
        targetLevel = level;
        if(targetLevel > Slides.MAX_LEVEL) {
            targetLevel = Slides.MAX_LEVEL;
        }
        if(targetLevel < 0){
            targetLevel = 0;
        }
    }

    public void runToLevel (){
        double targetLevelTicks = targetLevel * ENCODER_TICKS_PER_LEVEL;
        while (getCurrentPosition() < targetLevelTicks) {
            runAtPower(.5);
        }
        runAtPower(0);
    }

    /**
     * Enables auto control where target slide height is where we are now, freezing slides in position.
     * The more elegant alternative to setPower(0);
     */
//    public void freeze(){
//        freezeHeight = getCurrentPosition();
//        frozen = true;
//        startAutoControl();
//    }

    /**
     * Runs slide PID when auto control is enabled.
     * Package protected for unit testing.
     */
//    void autoControl(){
//        int target = getTargetHeight();
//        int current = getCurrentPosition();
//        int error = target - current; // in ticks
//
//        FTCUtilities.addData("target", target);
//        FTCUtilities.addData("current", current);
//        FTCUtilities.addData("error", current);
//
//        double correction = pid.getCorrection(error).totalCorrection;
//        motorPower += correction;
//        runAtPower(motorPower);

        /*if(Math.abs(error) > PID_ERROR_THRESHOLD) {
            if(Math.signum(error) == 1){ // go up
                runAtPower(UP_POWER/3);
            } else { // go down
                runAtPower(DOWN_POWER/3);
            }
            pid.trashIntegral(); // integral should stay at zero per 'pid sesh'
        } else { //do the pid thing

        }*/


//        FTCUtilities.addData("motor power", motorPower);
//        //FTCUtilities.addData("bottom", atBottom());
//        //FTCUtilities.addData("target level", targetLevel);
//        FTCUtilities.updateOpLogger();
//    }

    //USUALLY USE runAtPower(), it checks the slide limits.
    private void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void stopMotors() {
        setPower(0);
    }

    /**
     * @return returns average slide height calculated by motor encoders.
     */
    public int getCurrentPosition() {
        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition())/2; //note integer division
    }

    private class SlidesThread extends Thread {
        private volatile boolean running;
        private volatile  boolean finished;

        public SlidesThread() {
            running = false;
            finished = false;
        }

        private synchronized void kill(){
                finished = true;
                running = false;
                notifyAll();
        }

//        @Override
//        public void run() {
//            while(!finished){
//                while (running){
//                    autoControl();
//                }
//                if(!finished){
//                    try {
//                        synchronized (this) {
//                            wait();
//                        }
//                    } catch (Exception e){}
//                }
//            }
//        }
    }
}
