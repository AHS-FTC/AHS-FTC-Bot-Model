package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import edu.ahs.robotics.hardware.sensors.LimitSwitch;
import edu.ahs.robotics.util.FTCUtilities;

/**
 * Vertical slides system for robot. Encapsulates motors and limit switches
 */
public class Slides {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private LimitSwitch limitSwitch;
    private static final int ENCODER_TICKS_PER_LEVEL = 420;
    private static final int SLIDES_MAX = 4000;
    public static final int MAX_LEVEL = 10;
    private int targetLevel = 0;
    private static final double UP_POWER = .8;
    private static final double DOWN_POWER = -.1;

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

    /**
     * Sets the target level that you want the slides to achieve. Use runSlidesToTargetLevel() to execute
     * @param level desired height of slides in mm
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
        targetLevel = 0;
        while(!limitSwitch.isTriggered()) {
            setPower(DOWN_POWER);
        }
        stopMotors();
        resetEncoders();

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
    public double getCurrentPosition() {
        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition())/2.0;
    }

}
