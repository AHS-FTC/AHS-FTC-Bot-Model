/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.live;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import edu.ahs.robotics.hardware.ContinuosServo;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.Slides;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.ftc.FTCUtilities;
import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.util.ftc.Switch;
import edu.ahs.robotics.util.ftc.Toggle;


//Written by Alex Appleby of team 16896
//It really do be like that
//max at 4150 ticks
//first at 340
//min at 100

//Then edited by Andrew Seybold
//It really really do be like dat

@TeleOp(name="Ardennes TeleOp", group="Iterative Opmode")
//@Disabled
public class ArdennesTeleOp extends OpMode
{

    private enum IntakeMode{
        OFF,
        IN,
        OUT
    }

    private enum TapeMeasureMode {
        OFF,
        IN,
        OUT
    }

    private Intake intake;
    private Slides slides;
    private MecanumChassis chassis;

    private SerialServo gripper, capstone, ySlide, leftFoundation, rightFoundation;
    private ContinuosServo tapeMeasure;
    //todo add Servo capstoneServo;

    private TriggerDistanceSensor gripperTrigger, intakeTrigger;

    //from zero to one
    private double yServoPosition = 0;

    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double INTAKE_POWER = .5;
    private IntakeMode intakeMode = IntakeMode.OFF;
    private TapeMeasureMode tapeMeasureMode = TapeMeasureMode.OFF;
    private static final double SLIDE_DOWN_POWER_SCALE = 0.3; //unitless multiplier to weaken slide motors when pulling down

    private Toggle foundationToggle;
    private Toggle gripperToggle;
    private Toggle capstoneToggle;

    private Toggle collectionModeToggle;
    private Toggle debugToggle;
    private boolean slidesMoving = false;
    private boolean xPressed = false;
    private boolean runToLevelMode = false;

    private Switch tapeMeasureSwitchIn;
    private Switch tapeMeasureSwitchOut;

    private Switch intakeOutSwitch;
    private Switch intakeInSwitch;
    private int targetLevel = 0; //todo consider initializing

    private Switch slideControlSwitch;

    private ElapsedTime time;

    private double lastTime;

    @Override
    public void init() {

        FTCUtilities.setOpMode(this);
        time = new ElapsedTime();

        Ardennes ardennes = new Ardennes();

        slides = ardennes.getSlides();

        chassis = ardennes.getChassis();
        intake = ardennes.getIntake();

        gripper = ardennes.getGripper();
        capstone = ardennes.getCapstone();
        ySlide = ardennes.getySlide();
        leftFoundation = ardennes.getLeftFoundation();
        rightFoundation = ardennes.getRightFoundation();

        tapeMeasureSwitchIn = new Switch();
        tapeMeasureSwitchOut = new Switch();

        foundationToggle = new Toggle();
        gripperToggle = new Toggle();
        capstoneToggle = new Toggle();
        collectionModeToggle = new Toggle();
        debugToggle = new Toggle();

        intakeOutSwitch = new Switch();
        intakeInSwitch = new Switch();

        slideControlSwitch = new Switch();

        gripperTrigger = ardennes.getGripperTrigger();
        intakeTrigger = ardennes.getIntakeTrigger();

        gripper.setPosition(0);
        capstone.setPosition(0.22);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        time.startTime();
        lastTime = time.milliseconds();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        buttonActions();
        driveActions();
        slideActions();
        triggers();
    }

    private void slideActions() {

//        if(gamepad2.left_trigger >= TRIGGER_THRESHOLD || gamepad2.right_trigger >= TRIGGER_THRESHOLD){
//            if(slides.isAutoControlled()){
//                slides.stopAutoControl();
//            }
//            double slidesPower = gamepad2.right_trigger - (gamepad2.left_trigger * SLIDE_DOWN_POWER_SCALE);
//            slides.runAtPower(slidesPower);
//        } //else if(!slides.isAutoControlled()) { // if slides aren't in auto control, freeze them
//            slides.freeze();
//        //}
//
//        if(gamepad2.y){
//            if(slideControlSwitch.canFlip()){
//                targetLevel++;
//                slides.setTargetLevel(targetLevel);
//                slides.startAutoControl();
//            }
//        }
//        if(gamepad2.x){
//            if(slideControlSwitch.canFlip()){
//                targetLevel--;
//                slides.setTargetLevel(targetLevel);
//                slides.startAutoControl();
//            }
//        }


        //todo add a drop button

        //if either trigger is pressed then run the slides
        if(gamepad2.left_trigger >= TRIGGER_THRESHOLD || gamepad2.right_trigger >= TRIGGER_THRESHOLD) {
            slidesMoving = true;
            //if the slides are running to a level, cancel and run with manual control
            if(runToLevelMode){
                runToLevelMode = false;
            }
            //run slides at power
            double slidesPower = gamepad2.right_trigger - (gamepad2.left_trigger * SLIDE_DOWN_POWER_SCALE);
            slides.runAtPower(slidesPower);
        }

        //if slides are not in run to level mode and are not receiving inputs from triggers, stop motors
        if (gamepad2.left_trigger < TRIGGER_THRESHOLD && gamepad2.right_trigger < TRIGGER_THRESHOLD){
            if (!runToLevelMode && slidesMoving) {
                slides.stopMotors();
                slidesMoving = false;
            }
        }

        yServoPosition = Range.clip(yServoPosition + gamepad2.right_stick_y, 0, 1);
        ySlide.setPosition(yServoPosition);
//
//        //press x to increase levels for stacking
//        if (gamepad2.x) {
//            if (!xPressed) {
//                xPressed = true;
//                slides.incrementTargetLevel();
//            }
//        } else {
//            xPressed = false;
//        }
//
//        if (gamepad2.y) {
//            runToLevelMode = true;
//            slides.runSlidesToTargetLevel();
//        }
//
//        //press right bumper to reset slides to original position
//        if (gamepad2.right_bumper) {
//            if(gripperToggle.isEnabled()) {
//                gripperToggle.canFlip();
//                updateGripper();
//            }
//            if (capstoneToggle.isEnabled()) {
//                capstoneToggle.canFlip();
//                activateWrist();
//            }
//            yServoPosition = 0;
//            slides.setManualControlMode();
//            runToLevelMode = false;
//            ySlide.setPosition(yServoPosition);
//            FTCUtilities.sleep(500);
//            slides.resetSlidesToOriginalPosition();
//        }
    }

    private void triggers() {

        if (slides.atBottom()) {
            //Is the intake running but the gripper is closed? Open the gripper.
            if (intakeMode != IntakeMode.OFF && gripperToggle.isEnabled()) {
                gripperToggle.canFlip();
                updateGripper();
            }
            //Is the gripper distance sensor triggered? We have a block in position, stop the intake.
            if (gripperTrigger.isTriggered()) {
                if (intakeMode == IntakeMode.IN) {
                    intakeMode = IntakeMode.OFF;
                    updateIntake();
                }
                // Is the gripper open and in position and is the intake not running out? Then grip the block.
                if (!gripperToggle.isEnabled() && intakeMode != IntakeMode.OUT) {
                    gripperToggle.canFlip();
                    updateGripper();
                }
            }

        }

        // If in collection mode and a block is seen by the intake. Stop the motors but allow them to run outwards.
        if (intakeTrigger.isTriggered() && collectionModeToggle.isEnabled()) {
            if (intakeMode == IntakeMode.IN) {
                intakeMode = IntakeMode.OFF;
                updateIntake();
            }
        }
    }

    private void driveActions() {
        //Calculate motion components
        double forward = getClippedPower(-gamepad1.left_stick_y, .2);
        double strafe = Range.clip(getClippedPower(-gamepad1.left_stick_x, .2), -1, 1);
        double turn = getClippedPower(-gamepad1.right_stick_x, .2);
        chassis.drive3Axis(forward, strafe, turn);
    }

    private void buttonActions() {
        //press dpad down to enable debug logs
        if (gamepad1.dpad_down) {
            debugToggle.canFlip();
            if (debugToggle.isEnabled()) {
                //telemetry.addData("deltaTime",lastTime-time.milliseconds());
                //lastTime = time.milliseconds();
                telemetry.addData("y servo position", yServoPosition);
                telemetry.addData("limit switch triggered?", slides.atBottom());
                //telemetry.addData("intake trigger", intakeTrigger.isTriggered());
                //telemetry.addData("intake trigger distance", intakeTrigger.getDist());
                telemetry.update();
            } else {
                telemetry.clear();
            }
        }

        //press x on gamepad 1 to enable tapeMeasure
        if (gamepad1.x) {
            if (tapeMeasureSwitchOut.canFlip()) {
                if (tapeMeasureMode == TapeMeasureMode.OUT) {
                    tapeMeasureMode = TapeMeasureMode.OFF;
                } else {
                    tapeMeasureMode = TapeMeasureMode.OUT;
                }
                updateTapeMeasure();
            }
        }

        //press y on gamepad 1 to retract tape Measure
        if (gamepad1.y) {
            if (tapeMeasureSwitchIn.canFlip()) {
                if (tapeMeasureMode == TapeMeasureMode.IN) {
                    tapeMeasureMode = TapeMeasureMode.OFF;
                } else {
                    tapeMeasureMode = TapeMeasureMode.IN;
                }
                updateTapeMeasure();
            }
        }

        //press l bumper to reverse intake
        if (gamepad1.left_bumper) {
            if(intakeOutSwitch.canFlip()) {
                if (intakeMode == IntakeMode.OUT) {
                    intakeMode = IntakeMode.OFF;
                } else {
                    intakeMode = IntakeMode.OUT;
                }
                updateIntake();
            }
        }

        //press r bumper to enable intake
        if (gamepad1.right_bumper) {
            if (intakeInSwitch.canFlip()) {
                if (intakeMode == IntakeMode.IN) {
                    intakeMode = IntakeMode.OFF;
                } else {
                    intakeMode = IntakeMode.IN;
                }
                updateIntake();
            }
        }

        //press x to drop capstone
        if (gamepad2.x) {
            capstoneToggle.canFlip();
            activateCapstone();
        }

        //press a to grip block
        if (gamepad2.a) {
            gripperToggle.canFlip();
            updateGripper();
        }

        //press a to grip foundation
        if (gamepad1.a) {
            foundationToggle.canFlip();
            if (foundationToggle.isEnabled()) {
                leftFoundation.setPosition(1);
                rightFoundation.setPosition(1);
            } else {
                leftFoundation.setPosition(0);
               rightFoundation.setPosition(0);
            }
        }

        //gamepad 1 press dpad up to enable collection mode
        if (gamepad1.dpad_up) {
            collectionModeToggle.canFlip();
            telemetry.addData("Collection Mode?", collectionModeToggle.isEnabled());
            //telemetry.update();
        }

    }

    private void activateCapstone() {
        if (capstoneToggle.isEnabled()) {
            capstone.setPosition(1);
        } else {
            capstone.setPosition(0.22);
        }
    }

    private void updateTapeMeasure() {
        if (tapeMeasureMode == tapeMeasureMode.OUT) {
            tapeMeasure.setPower(.85);
        } else if (tapeMeasureMode == tapeMeasureMode.OFF) {
            tapeMeasure.setPower(0);
        } else if (tapeMeasureMode == TapeMeasureMode.IN) {
            tapeMeasure.setPower(-.85);
        }
    }

    private void updateIntake() {
        if(intakeMode == IntakeMode.IN){
            intake.runMotors(INTAKE_POWER);
        } else if (intakeMode == IntakeMode.OFF) {
            intake.stopMotors();
        } else if (intakeMode == IntakeMode.OUT) {
            intake.runMotors(-INTAKE_POWER);
        }
    }

    private void updateGripper() {
        if(gripperToggle.isEnabled()){
            gripper.setPosition(1);
        } else {
            gripper.setPosition(0);
        }
        telemetry.addData("Gripper Toggled?", gripperToggle.isEnabled());
        //telemetry.update();
    }

    private double getClippedPower(double input, double minPower){
        final double ZERO_RANGE = 0.01; //inputs beneath this range are ignored
        //final double MIN_POWER = 0.2; // lowest(ish) power in which robot still moves

        if(Math.abs(input) <= ZERO_RANGE){
            return 0;
        } else {
            return Math.signum(input)*Math.max(minPower,Math.abs(input));//abs to legitimize max, signum to retain directionality
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        chassis.stopMotors();
        intake.stopMotors();
//        slides.kill();
        slides.stopMotors();
    }
}
