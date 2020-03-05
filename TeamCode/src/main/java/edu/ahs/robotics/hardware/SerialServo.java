package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import edu.ahs.robotics.util.ftc.FTCUtilities;

/**
 * Wrapper class for FTCApp servo. Adds additional functionality.
 * @author Alex Appleby
 * @see Servo
 */
public class SerialServo {
    private Servo servo;

    private TimeControlState state;

    private long timeControlDuration = 1000;
    private long startTime;

    private double timeControlStart;
    private double timeControlTarget;
    private double timeControlGradient; //the difference between the start and target

    private enum TimeControlState {
        STARTED,
        RUNNING,
        STOPPED
    }

    public SerialServo(String deviceName, boolean reverse) {
        servo = FTCUtilities.getSerialServo(deviceName);
        if(reverse){
            servo.setDirection(Servo.Direction.REVERSE);
        } else {
            servo.setDirection(Servo.Direction.FORWARD);
        }
        state = TimeControlState.STOPPED;
    }

    public void setPosition(double position){
        servo.setPosition(position);
    }

    public void mapPosition(double min, double max){
        servo.scaleRange(min, max);
    }

    /**
     * Sets the amount of time it takes to run this servo under timeControl operation.
     * @param timeControlDuration
     */
    public void setTimeControlDuration(long timeControlDuration){
        this.timeControlDuration = timeControlDuration;
    }

    public void setTimeControlTarget(double target){
        timeControlTarget = target;
    }

    /**
     * Runs a servo from it's current position to a specified target position over a set interval of time.
     */
    public void runWithTimeControl(){
        FTCUtilities.addData("state", state);

        switch (state){
            case STOPPED:
                break;
            case STARTED:
                startTime = FTCUtilities.getCurrentTimeMillis();

                timeControlStart = servo.getPosition();
                timeControlGradient = timeControlTarget - timeControlStart;

                state = TimeControlState.RUNNING;
                break;
            case RUNNING:
                long elapsedTime = FTCUtilities.getCurrentTimeMillis() - startTime;
                double timePercent = (double) elapsedTime / (double) timeControlDuration; // what amount of the total time have we elapsed

                FTCUtilities.addData("time percent", timePercent);

                if(timePercent >= 1.0){
                    servo.setPosition(timeControlTarget);
                    state = TimeControlState.STOPPED;
                } else {
                    double position = timeControlStart + (timeControlGradient * timePercent);
                    FTCUtilities.addData("position", position);
                    servo.setPosition(position);
                }
                break;
        }
        FTCUtilities.updateOpLogger();

    }

    /**
     * Must be called to reset time control state after a full cycle.
     */
    public void restartTimeControl(){
        state = TimeControlState.STARTED;
    }

//    public void breakTimeControl(){
//        state = TimeControlState.STOPPED;
//    }

    public boolean isTimeControlRunning(){
        return state == TimeControlState.RUNNING;
    }
}
