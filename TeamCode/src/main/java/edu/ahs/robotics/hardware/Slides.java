package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import edu.ahs.robotics.hardware.sensors.LimitSwitch;
import edu.ahs.robotics.util.ftc.FTCUtilities;
import edu.ahs.robotics.util.ftc.Switch;

/**
 * Vertical slides system for robot. Encapsulates motors and limit switches
 */
public class Slides {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private LimitSwitch limitSwitch;
    private LimitSwitch limitSwitch2;

    private int targetLevel = 0;

    private static final int ENCODER_TICKS_PER_LEVEL = 420;
    private static final int SLIDES_MAX = 4000; // maximum encoder val of slides
    private static final int MAX_LEVEL = 10;

    /**
     * 'P' terms used for holding slide position. Separated to account for gravitational bias. Will require tuning.
     */
    //private static final double UP_CORRECTION = 0.015; //todo tune
    //private static final double DOWN_CORRECTION = 0.01;

    private double upCorrection = 0.015;
    private double downCorrection = 0.01;

    /**
     * Power that slides return with using auto retraction
     */
    private static final double RETURN_POWER = -.3;

    private double gamepadControlPower = 0.0;
    private Gamepad gamepad;
    private State state;

    private Switch returnSwitch, autoControlSwitch, levelSwitch;
    private double autoControlPower = 0.0;

    private int freezePosition = 0;

    /**
     * Target encoder height for each level
     */
    private static int[] levelHeights = { //todo tune
            0,   //0
            100, //1
            200, //2
            300, //3
            400, //4
            500, //5
            600, //6
            700, //7
            800, //8
            900, //9
            1000, //10
            1100, //11
            1200, //12
            1300, //13
            1400, //14
            1500, //15
    };

    private enum State {

        /**
         * Controlled by the inputs of a GamePad.
         */
        USER_CONTROLLED,

        /**
         * Controlled by auto levelling software.
         */
        AUTO_CONTROLLED,

        /**
         * Auto levelled to position driven to when user controlled.
         */
        FROZEN,

        /**
         * Holding negative power until interrupted or triggering limit switches.
         */
        RUNNING_TO_BOTTOM,

        /**
         * Sitting at bottom waiting for inputs
         */
        AT_BOTTOM
    }

    public Slides (){
        leftMotor = FTCUtilities.getMotor("slideL");
        rightMotor = FTCUtilities.getMotor("slideR");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limitSwitch = new LimitSwitch("limitSwitch", true);
        limitSwitch2 = new LimitSwitch("limitSwitch2", true);

        returnSwitch = new Switch();
        autoControlSwitch = new Switch();
        levelSwitch = new Switch();

        state = State.AT_BOTTOM;
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
        }

        setPower(slidesPower);
    }

    /**
     * @return true if slides are at bottom
     */
    public boolean atBottom(){
        return (limitSwitch.isTriggered() || limitSwitch2.isTriggered());
    }

    public void resetEncoders(){
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
     * Calculates the power from auto control. Does proportional corrections.
     * @param targetTick The target height of the slides.
     */
    private void calculateAutoControlPower(int targetTick){
        int currentTick = getCurrentPosition();

        int error = targetTick - currentTick; //positive = below and correcting up

        if(Math.signum(error) == 1){
            autoControlPower += (upCorrection * error);
        } else {
            autoControlPower += (downCorrection * error);
        }
    }

    /**
     * Runs slide PID when auto control is enabled.
     * Package protected for unit testing.
     */

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

    /**
     * Controls the slides with the gamepad.
     * Nonblocking, use in an iterative context.
     */
    public void gamepadControl(){

        checkInputs();
        switch (state){
            case AT_BOTTOM:
                break;
            case AUTO_CONTROLLED:
                calculateAutoControlPower(levelHeights[targetLevel]);
                runAtPower(autoControlPower);
                break;
            case RUNNING_TO_BOTTOM:
                if(atBottom()){
                    stopMotors();
                    state = State.AT_BOTTOM;
                }
                break;
            case FROZEN:
                calculateAutoControlPower(freezePosition);
                runAtPower(autoControlPower);
                break;
            case USER_CONTROLLED:
                runAtPower(gamepad.right_trigger - gamepad.left_trigger);
                break;
        }
    }

    /**
     * Allows you to directly tune the slide corrections. Comment this out when you're done.
     */
    public void tuningPIDControl(double upCorrection, double downCorrection){
        this.upCorrection = upCorrection;
        this.downCorrection = downCorrection;

        calculateAutoControlPower(1000);
        if (gamepad.x){
            stopMotors();
        } else {
            setPower(autoControlPower);
        }
    }

    /**
     * Sets the gamepad that controls the slides in gamepadControl();
     */
    public void setGamepad(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    /**
     * Resets auto control power, throwing away old corrections.
     */
    private void resetAutoControlPower(){
        autoControlPower = 0.0;
    }

    private void raiseTargetLevel(){
        if(targetLevel < levelHeights.length){
            targetLevel++;
        }
    }

    private void lowerTargetLevel(){
        if(targetLevel > 0){
            targetLevel--;
        }
    }

    /**
     * Check controller inputs to determine slide state
     */
    private void checkInputs(){
        if(gamepad.left_trigger != 0.0 || gamepad.right_trigger != 0.0){

            //if there's any trigger control, escape any automation and go into user control
            state = State.USER_CONTROLLED;

        } else if (gamepad.y && returnSwitch.canFlip()){

            state = State.RUNNING_TO_BOTTOM;
            runAtPower(RETURN_POWER);

        } else if (gamepad.b && autoControlSwitch.canFlip()){

            state = State.AUTO_CONTROLLED;
            raiseTargetLevel();
            resetAutoControlPower();

        } else if (state == State.USER_CONTROLLED){ //if there are no inputs and the state is user controlled, freeze slides
            freezePosition = getCurrentPosition();
            state = State.FROZEN;
            resetAutoControlPower();
        } //if none of these things are true, maintain state.

        if (gamepad.dpad_up && levelSwitch.canFlip()){ //check up / down level controls
            raiseTargetLevel();
        } else if (gamepad.dpad_down && levelSwitch.canFlip()){
            lowerTargetLevel();
        }
    }
}
