package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import edu.ahs.robotics.control.pid.PID;
import edu.ahs.robotics.hardware.sensors.LimitSwitch;
import edu.ahs.robotics.util.FTCUtilities;

/**
 * Vertical slides system for robot. Encapsulates motors and limit switches
 */
public class Slides {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private LimitSwitch limitSwitch;

    private double motorPower = 0;
    private int targetLevel = 0;

    private static final int ENCODER_TICKS_PER_LEVEL = 420;
    private static final int PID_ERROR_THRESHOLD = 30;
    private static final int SLIDES_MAX = 4000; // maximum encoder val of slides
    private static final int MAX_LEVEL = 10;
    private static final double UP_POWER = 1;
    private static final double DOWN_POWER = -.1;

    private static final double P = 0;
    private static final double I = 0;
    private static final double D = 0;

    private PID pid = new PID(P,I,D);

    private SlidesThread thread = new SlidesThread();



    public Slides (){
        leftMotor = FTCUtilities.getMotor("slideL");
        rightMotor = FTCUtilities.getMotor("slideR");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setManualControlMode();

        limitSwitch = new LimitSwitch("limitSwitch");
    }

    public void goToBottom(){
        targetLevel = 0;
    }


    /**
     * Direct control over slide motors at a specified power
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
        return limitSwitch.isTriggered();
    }

    public void resetEncoders(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setEncoderModeRunToPostion(){
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setManualControlMode() {
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startAutoControl(){
        thread.run();
    }

    public void stopAutoControl(){
        thread.end();
    }

    private int getTargetHeight(){
        return targetLevel * ENCODER_TICKS_PER_LEVEL;
    }

    /**
     * Sets the target level that you want the slides to achieve.
     * @param level Number of blocks high you wish to raise slides to.
     */
    public void setTargetLevel (int level) {
        targetLevel = level;
        if(targetLevel > Slides.MAX_LEVEL) {
            targetLevel = Slides.MAX_LEVEL;
        }
    }

    public void incrementTargetLevel () {
        setTargetLevel(targetLevel+1);
    }

    /**
     * Moves the slides to the level specified by incrementTargetLevel() or setTargetLevel.
     * Nonblocking, returns immediately
     */
    public void runSlidesToTargetLevel() {
        int level1 = 200;
        int ticksAtLevel;
        ticksAtLevel = (targetLevel-1) * ENCODER_TICKS_PER_LEVEL + level1;
        leftMotor.setTargetPosition(ticksAtLevel);
        rightMotor.setTargetPosition(ticksAtLevel);
        setEncoderModeRunToPostion();
        setPower(UP_POWER);
    }

    /**
     * Retracts slides down to origin. Blocks until slides at origin
     */
    public void resetSlidesToOriginalPosition() {
        setManualControlMode();
        targetLevel = 0;
        while(!atBottom()) {
            setPower(DOWN_POWER);
        }
        stopMotors();
        resetEncoders();

    }

    /**
     * Runs slide PID when auto control is enabled.
     * Package protected for unit testing.
     */
    void autoControl(){
        int target = getTargetHeight();
        int current = getCurrentPosition();
        int error = target - current; // in ticks

        if(Math.abs(error) > PID_ERROR_THRESHOLD) {
            if(Math.signum(error) == 1){ // go up
                setPower(UP_POWER);
            } else { // go down
                setPower(DOWN_POWER);
            }
            pid.trashIntegral(); // integral should stay at zero per 'pid sesh'
        } else { //do the pid thing
            double correction = pid.getCorrection(target,current);
            motorPower += correction;
            setPower(motorPower);
        }
    }

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
    private int getCurrentPosition() {
        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition())/2; //note integer division
    }

    private class SlidesThread extends Thread {
        private volatile boolean running;

        private SlidesThread() {
            running = false;
        }

        private void end(){
            running = false;
        }

        @Override
        public void run() {
            running = true;
            while(running){
                autoControl();
            }
        }
    }
}
