package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private LimitSwitch limitSwitch;
    private LimitSwitch limitSwitch2;

    private int targetLevel = 0;

    private static final int SLIDES_MAX = 4000; // maximum encoder val of slides
    private static final int MAX_LEVEL = 10;

    /**
     * Power that slides return with using auto retraction
     */
    private static final double DOWN_POWER = -.3;

    private Gamepad gamepad;
    private State state;

    private Switch returnSwitch, autoControlSwitch, levelSwitch;

    private int freezePosition = 0;

    /**
     * Target encoder height for each level
     */
    private static int[] levelHeights = { //tuned
            0,    //0
            300,  //1
            580,  //2
            895,  //3
            1170, //4
            1495, //5
            1775, //6
            2065, //7
            2365, //8
            2665, //9
            2935, //10
            3255, //11
            3540, //12
            3860  //13
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

    /**
     * For small state machine that controls drive to height
     */

    public Slides (){
        leftMotor = FTCUtilities.getMotorEx("slideL");
        rightMotor = FTCUtilities.getMotorEx("slideR");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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
        if(leftMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

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
//        FTCUtilities.addData("mode", state);
//        FTCUtilities.addData("encoder reading", getCurrentPosition());
//        FTCUtilities.addData("encoder reading L", leftMotor.getCurrentPosition());
//        FTCUtilities.addData("encoder reading R", rightMotor.getCurrentPosition());
//        FTCUtilities.addData("target level", targetLevel);
//
//        FTCUtilities.updateOpLogger();

        checkInputs();
        switch (state){
            case AT_BOTTOM: //todo change this state
                break;
            case AUTO_CONTROLLED:
                autoControl(levelHeights[targetLevel]);
                break;
            case RUNNING_TO_BOTTOM:
                if(atBottom()){
                    stopMotors();
                    state = State.AT_BOTTOM;
                }
                break;
            case FROZEN:
                autoControl(freezePosition);
                break;
            case USER_CONTROLLED:
                runAtPower(gamepad.right_trigger - gamepad.left_trigger);
                break;
        }
    }

    /**
     * Sets the gamepad that controls the slides in gamepadControl();
     */
    public void setGamepad(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    private void raiseTargetLevel(){
        if(targetLevel < levelHeights.length){
            targetLevel++;
        }
    }

    private void autoControl(int targetPosition){
        if(leftMotor.getTargetPosition() != targetPosition){
            setTargetPositions(targetPosition);
        }

        if(leftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        leftMotor.setPower(.7);
        rightMotor.setPower(.7);
    }

    private void lowerTargetLevel(){
        if(targetLevel > 0){
            targetLevel--;
        }
    }

    private void setTargetPositions(int position){
        leftMotor.setTargetPosition(position);
        rightMotor.setTargetPosition(position);
    }

    /**
     * Check controller inputs to determine slide state
     */
    private void checkInputs(){
        if(gamepad.left_trigger != 0.0 || gamepad.right_trigger != 0.0){

            //if there's any trigger control, escape any automation and go into user control
            state = State.USER_CONTROLLED;

        } else if (gamepad.b && returnSwitch.canFlip()){

            state = State.RUNNING_TO_BOTTOM;
            runAtPower(DOWN_POWER);

        } else if (gamepad.y && autoControlSwitch.canFlip()){

            state = State.AUTO_CONTROLLED;
            raiseTargetLevel();

        } else if (state == State.USER_CONTROLLED){ //if there are no inputs and the state is user controlled, freeze slides

            freezePosition = getCurrentPosition();
            state = State.FROZEN;

        } //if none of these things are true, maintain state.

        if (gamepad.dpad_up && levelSwitch.canFlip()){ //check up / down level controls
            raiseTargetLevel();
        } else if (gamepad.dpad_down && levelSwitch.canFlip()){
            lowerTargetLevel();
        }
    }
}
