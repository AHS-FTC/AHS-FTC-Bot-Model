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
 * ARE DISCLAIMED. INSLOW NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER INSLOW CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING INSLOW ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.live;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import edu.ahs.robotics.hardware.Blinkin;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.Slides;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.util.ftc.Switch;
import edu.ahs.robotics.util.ftc.Toggle;
import edu.ahs.robotics.util.opmodes.bfr.IterativeOpMode16896;


/**
 * Main TeleOp for Ardennes 2019-2020
 * @author Alex Appleby and Andrew Seybold
 */
@TeleOp(name="Ardennes TeleOp", group="Iterative Opmode")
//@Disabled
public class ArdennesTeleOp extends IterativeOpMode16896
{

    private enum IntakeMode{
        OFF,
        INSLOW,
        OUT,
        INFAST,
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
    //todo add Servo capstoneServo;

    private Blinkin blinkin;

    private TriggerDistanceSensor gripperTrigger, intakeTrigger;

    //from zero to one
    private double yServoPosition = 0;

    private static final double INTAKE_POWER = .5;
    private IntakeMode intakeMode = IntakeMode.OFF;
    private TapeMeasureMode tapeMeasureMode = TapeMeasureMode.OFF;

    private Toggle foundationToggle;
    private Toggle gripperToggle;
    private Toggle capstoneToggle;

    private Toggle collectionModeToggle;
    private Toggle debugToggle;

    private Switch intakeOutSwitch;
    private Switch intakeInSwitch;

    private Switch slideControlSwitch;

    private ElapsedTime time;

    private double lastTime;

    @Override
    public void initialize() {
        time = new ElapsedTime();

        Ardennes ardennes = new Ardennes();

        slides = ardennes.getSlides();
        slides.setGamepad(gamepad2);

        chassis = ardennes.getChassis();
        intake = ardennes.getIntake();

        gripper = ardennes.getGripper();
        capstone = ardennes.getCapstone();
        ySlide = ardennes.getxSlide();
        leftFoundation = ardennes.getLeftFoundation();
        rightFoundation = ardennes.getRightFoundation();

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

        blinkin = ardennes.getBlinkin();

        gripper.setPosition(0);
        capstone.setPosition(0.22);
        ySlide.mapPosition(.3,.75);
    }

    @Override
    public void repeatAfterInit() {
    }

    @Override
    public void begin() {
        time.startTime();
        lastTime = time.milliseconds();
    }

    @Override
    public void iterate() {
        buttonActions();
        driveActions();
        slideActions();
        triggers();
        updateBlinkin();
    }

    private void slideActions() {
        slides.gamepadControl();

        yServoPosition = Range.clip(yServoPosition + gamepad2.right_stick_y, 0, 1);
        ySlide.setPosition(yServoPosition);
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
                if (intakeMode == IntakeMode.INSLOW || intakeMode == IntakeMode.INFAST) {
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
            if (intakeMode == IntakeMode.INSLOW) {
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

        //press r bumper to enable slow intake
        if (gamepad1.right_bumper) {
            if (intakeInSwitch.canFlip()) {
                if (intakeMode == IntakeMode.INSLOW) {
                    intakeMode = IntakeMode.OFF;
                } else {
                    intakeMode = IntakeMode.INSLOW;
                }
                updateIntake();
            }
        }

        //use trigger to speed up intake
        if(gamepad1.right_trigger != 0) {
            intakeMode = IntakeMode.INFAST;
            updateIntake();
        } else {
            if (intakeMode == IntakeMode.INFAST) {
                intakeMode = IntakeMode.INSLOW;
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

    private void updateBlinkin() {
        if (((gripperToggle.isEnabled() && slides.atBottom()) && yServoPosition <.2)){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            return;
        } else if (gripperTrigger.isTriggered() || intakeTrigger.isTriggered()){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            return;
        } else if ((intakeMode == IntakeMode.INFAST || intakeMode == IntakeMode.INSLOW)){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            return;
        } else if (collectionModeToggle.isEnabled()){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            return;
        } else{
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            return;
        }

    }

    private void updateIntake() {
        if(intakeMode == IntakeMode.INSLOW){
            intake.runMotors(INTAKE_POWER);
        } else if (intakeMode == IntakeMode.OFF) {
            intake.stopMotors();
        } else if (intakeMode == IntakeMode.OUT) {
            intake.runMotors(-INTAKE_POWER);
        } else if (intakeMode == IntakeMode.INFAST) {
            double fastPower = Range.clip((gamepad1.right_trigger + .5), .5, 1);
            intake.runMotors(fastPower);
        }
    }

    private void updateGripper() {
        if(gripperToggle.isEnabled()){
            gripper.setPosition(1);
            updateBlinkin();
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

    @Override
    public void teardown() {
        chassis.stopMotors();
        intake.stopMotors();
        slides.stopMotors();
    }
}
