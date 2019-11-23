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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import edu.ahs.robotics.hardware.sensors.TriggerDistanceSenor;
import edu.ahs.robotics.util.FTCUtilities;

//Written by Alex Appleby of team 16896
//It really do be like that
//max at 4150 ticks
//first at 340
//min at 100


@TeleOp(name="Ardennes TeleOp", group="Iterative Opmode")
//@Disabled
public class ArdennesTeleOp extends OpMode
{
    private enum IntakeMode{
        OFF,
        IN,
        OUT
    }

    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor intakeL, intakeR;
    DcMotor slideL, slideR;

    Servo ySlideServo;
    Servo foundationServoL, foundationServoR;
    Servo gripperServo, wristServo;

    //Servo capstoneServo;
    TriggerDistanceSenor intakeTrigger;
    TriggerDistanceSenor gripperTrigger;

    private double frontLeftPower = 0, frontRightPower = 0, backLeftPower = 0, backRightPower = 0;
    private final int SLIDES_MAX = 4150;
    private final int SLIDES_MIN = 100;
    private final int SLIDES_GRIP_THRESHOLD = 120;

    //from zero to one
    private double yServoPosition = 0;
    private final double Y_SERVO_SPEED = .02;//unitless multiplier - only effects speed attached to stick.

    //private double intakeLPower = 0, intakeRPower = 0;
    private final double INTAKE_POWER = 1;
    private IntakeMode intakeMode = IntakeMode.OFF;

    private double slideLPower = 0, slideRPower = 0;
    private final double SLIDE_DOWN_POWER_SCALE = 0.3; //unitless multiplier to weaken slide motors when pulling down

    private boolean debuggingEnabled = false;
    private boolean isDeliveryIntakeStyle = false;
    private boolean wristEnabled = false;
    private boolean gripperEnabled = false;

    private ElapsedTime time;

    private final double BUTTON_THRESHOLD = 300; //in millis - time between presses
    private double lastDebugPress = -BUTTON_THRESHOLD;//should be pressable at start
    private double lastDeliveryIntakeStyle = -BUTTON_THRESHOLD;
    private double lastIntakeInPress = -BUTTON_THRESHOLD;
    private double lastIntakeOutPress = -BUTTON_THRESHOLD;
    private double lastWristPress = -BUTTON_THRESHOLD;
    private double lastGripperPress = -BUTTON_THRESHOLD;

    //todo add serbos

    @Override
    public void init() {

        FTCUtilities.setHardwareMap(hardwareMap);
        FTCUtilities.setOpMode(this);

        frontLeft = hardwareMap.get(DcMotor.class,"FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        intakeL = hardwareMap.get(DcMotor.class,"intakeL");
        intakeR = hardwareMap.get(DcMotor.class,"intakeR");

        slideL = hardwareMap.get(DcMotor.class,"slideL");
        slideR = hardwareMap.get(DcMotor.class,"slideR");

        ySlideServo = hardwareMap.get(Servo.class,"slideServo");

        foundationServoL = hardwareMap.get(Servo.class,"FSL");
        foundationServoR = hardwareMap.get(Servo.class,"FSR");

        gripperServo = hardwareMap.get(Servo.class,"gripper");
        wristServo = hardwareMap.get(Servo.class,"wrist");

        gripperTrigger = new TriggerDistanceSenor("gripperTrigger",70);
        intakeTrigger = new TriggerDistanceSenor("intakeTrigger",40);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeR.setDirection(DcMotorSimple.Direction.FORWARD);

        slideL.setDirection(DcMotorSimple.Direction.FORWARD);
        slideR.setDirection(DcMotorSimple.Direction.REVERSE);

        ySlideServo.setDirection(Servo.Direction.FORWARD);

        foundationServoL.setDirection(Servo.Direction.FORWARD);
        foundationServoR.setDirection(Servo.Direction.FORWARD);

        gripperServo.setDirection(Servo.Direction.FORWARD);
        wristServo.setDirection(Servo.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        time = new ElapsedTime();

        telemetry.addLine("initialized");
        telemetry.update();

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

        resetEncoder(slideL);
        resetEncoder(slideR);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //press dpad down to enable debug logs
        if(gamepad1.dpad_down) {
            if(time.milliseconds() - lastDebugPress > BUTTON_THRESHOLD) {
                debuggingEnabled = !debuggingEnabled;
                lastDebugPress = time.milliseconds();
                resetEncoder(slideL);
                resetEncoder(slideR);
            }
        }

        //press l bumper to reverse intake
        if(gamepad1.left_bumper) {
            if(time.milliseconds() - lastIntakeOutPress > BUTTON_THRESHOLD) {
                if(intakeMode == IntakeMode.OUT) {
                    intakeMode = IntakeMode.OFF;
                } else {
                    intakeMode = IntakeMode.OUT;
                }
                lastIntakeOutPress = time.milliseconds();
            }
        }

        //press r bumper to enable intake
        if(gamepad1.right_bumper) {
            if(time.milliseconds() - lastIntakeInPress > BUTTON_THRESHOLD) {
                if(intakeMode == IntakeMode.IN) {
                    intakeMode = IntakeMode.OFF;
                } else {
                    intakeMode = IntakeMode.IN;
                }
                lastIntakeInPress = time.milliseconds();
            }
        }

        //press b to rotate stone 90 degrees
        if(gamepad2.b) {
            if(time.milliseconds() - lastWristPress > BUTTON_THRESHOLD) {
                wristEnabled = !wristEnabled;
                lastWristPress = time.milliseconds();
            }
        }

        //press a to grip block
        if(gamepad2.a) {
            if(time.milliseconds() - lastGripperPress > BUTTON_THRESHOLD) {
                gripperEnabled = !gripperEnabled;
                lastGripperPress = time.milliseconds();
            }
        }

        //press driver r bumper to toggle grippers
        if(gamepad1.right_bumper) {
            if(time.milliseconds() - lastGripperPress > BUTTON_THRESHOLD) {
                gripperEnabled = !gripperEnabled;
                lastGripperPress = time.milliseconds();
            }
        }

        //both controllers press dpad up to enable delivery style intake
        if(gamepad1.dpad_up && gamepad2.dpad_up) {
            if(time.milliseconds() - lastDeliveryIntakeStyle > BUTTON_THRESHOLD) {
                isDeliveryIntakeStyle = !isDeliveryIntakeStyle;
                lastDeliveryIntakeStyle = time.milliseconds();
            }
        }

        //negatives are important here
        frontLeftPower = gamepad1.left_stick_y+gamepad1.left_stick_x-gamepad1.right_stick_x;
        frontRightPower = gamepad1.left_stick_y-gamepad1.left_stick_x+gamepad1.right_stick_x;
        backLeftPower = gamepad1.left_stick_y-gamepad1.left_stick_x-gamepad1.right_stick_x;
        backRightPower = gamepad1.left_stick_y+gamepad1.left_stick_x+gamepad1.right_stick_x;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        slideLPower = gamepad1.left_trigger-(gamepad1.right_trigger*SLIDE_DOWN_POWER_SCALE);

        if(slideL.getCurrentPosition() >= SLIDES_MAX){
            slideLPower = Range.clip(slideLPower, -1, 0);
        } else if(slideL.getCurrentPosition()<= SLIDES_MIN){
            slideLPower = Range.clip(slideLPower, 0, 1);
        }
        slideRPower = slideLPower;

        slideL.setPower(slideLPower);
        slideR.setPower(slideRPower);

        yServoPosition = Range.clip(yServoPosition + gamepad2.right_stick_y*Y_SERVO_SPEED, 0,1);
        ySlideServo.setPosition(yServoPosition);

        if(isDeliveryIntakeStyle && intakeMode == IntakeMode.IN && intakeTrigger.isTriggered()){//if playing delivery, stop intake at trigger.
            intakeMode = IntakeMode.OFF;
        }

        if(intakeMode == IntakeMode.IN){
            intakeL.setPower(INTAKE_POWER);
            intakeR.setPower(INTAKE_POWER);
        } else if (intakeMode == IntakeMode.OFF) {
            intakeL.setPower(0);
            intakeR.setPower(0);
        } else if(intakeMode == intakeMode.OUT){
            intakeL.setPower(-INTAKE_POWER);
            intakeR.setPower(-INTAKE_POWER);
        }

        if(wristEnabled){
            wristServo.setPosition(1);
        } else {
            wristServo.setPosition(0);
        }
        //if the gripperTrigger is flipped while the gripper is disabled and in position to grip.
        if(!gripperEnabled && gripperTrigger.isTriggered() && slideL.getCurrentPosition()<= SLIDES_GRIP_THRESHOLD){
            gripperEnabled = true;
        }

        if(gripperEnabled){
            gripperServo.setPosition(1);
        } else {
            gripperServo.setPosition(0);
        }

        if(debuggingEnabled){
            telemetry.addData("Left Slide Encoder", slideL.getCurrentPosition());
            //telemetry.addData("Right Slide Encoder", slideR.getCurrentPosition());

            //telemetry.addData("intake Distance", intakeTrigger.getDist());
            telemetry.addData("intake triggered?", intakeTrigger.isTriggered());
            //telemetry.addData("gripper targetDistance", gripperTrigger.getDist());
            telemetry.addData("gripper triggered?", gripperTrigger.isTriggered());


            telemetry.update();
        } else {
            telemetry.clear();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);

        backRight.setPower(0);

        intakeL.setPower(0);
        intakeR.setPower(0);

        slideL.setPower(0);
        slideR.setPower(0);
    }

    public void resetEncoder(DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}