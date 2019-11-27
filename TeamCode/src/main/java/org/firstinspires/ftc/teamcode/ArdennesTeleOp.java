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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.Slides;
import edu.ahs.robotics.hardware.sensors.LimitSwitch;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.hardware.Intake;

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

    private Intake intake;
    private Slides slides;
    private MecanumChassis chassis;

    private SerialServo gripper, wrist, ySlide, leftFoundation, rightFoundation;
    //todo add Servo capstoneServo;

    private TriggerDistanceSensor gripperTrigger, intakeTrigger;

    //from zero to one
    private double yServoPosition = 0;

    private final double INTAKE_POWER = 1;
    private IntakeMode intakeMode = IntakeMode.OFF;

    private final double SLIDE_DOWN_POWER_SCALE = 0.3; //unitless multiplier to weaken slide motors when pulling down

    private Toggle foundationToggle;
    private Toggle gripperToggle;
    private Toggle wristToggle;
    private Toggle collectionModeToggle;
    private Toggle debugToggle;

    private Switch intakeOutSwitch;
    private Switch intakeInSwitch;

    private ElapsedTime time;

    private double lastTime;

    //todo add serbos

    @Override
    public void init() {
        FTCUtilities.setOpMode(this);
        time = new ElapsedTime();

        Ardennes ardennes = new Ardennes();

        chassis = ardennes.getChassis();
        intake = ardennes.getIntake();
        slides = ardennes.getSlides();

        gripper = ardennes.getGripper();
        wrist = ardennes.getWrist();
        ySlide = ardennes.getySlide();
        leftFoundation = ardennes.getLeftFoundation();
        rightFoundation = ardennes.getRightFoundation();

        foundationToggle = new Toggle();
        gripperToggle = new Toggle();
        wristToggle = new Toggle();
        collectionModeToggle = new Toggle();
        debugToggle = new Toggle();

        intakeOutSwitch = new Switch();
        intakeInSwitch = new Switch();

        gripperTrigger = ardennes.getGripperTrigger();
        intakeTrigger = ardennes.getIntakeTrigger();

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
        lastTime = time.milliseconds();

        slides.resetEncoders();
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
        double slidesPower = gamepad2.right_trigger - (gamepad2.left_trigger * SLIDE_DOWN_POWER_SCALE);

        slides.runSlides(slidesPower);

        yServoPosition = Range.clip(yServoPosition + gamepad2.right_stick_y, 0, 1);
        ySlide.setPosition(yServoPosition);
    }

    private void triggers() {
        // If the gripperTrigger is flipped while the gripper is disabled and in position to grip.
        if (gripperTrigger.isTriggered()) {
            intakeMode = IntakeMode.OFF;
            updateIntake();
            // Is the gripper open and in position?
            if (!gripperToggle.isEnabled() && slides.atBottom()) {
                gripperToggle.flip();
                activateGripper();
            }
        }
    }

    private void driveActions() {
        //Calculate motion components
        double forward = getClippedPower(-gamepad1.left_stick_y, .2);
        double strafe = Range.clip(getClippedPower(-gamepad1.left_stick_x, .2), -.4, .4);
        double turn = getClippedPower(Math.pow(-gamepad1.right_stick_x, 3), .25);
        chassis.drive3Axis(forward, strafe, turn);
    }

    private void buttonActions() {
        //press dpad down to enable debug logs
        if (gamepad1.dpad_down) {
            debugToggle.flip();
            if (debugToggle.isEnabled()) {
                telemetry.addData("deltaTime",lastTime-time.milliseconds());
                lastTime = time.milliseconds();
                telemetry.addData("Slides Average Encoder Value", slides.getCurrentPosition());
                telemetry.addData("y servo position", yServoPosition);

                telemetry.update();
            } else {
                telemetry.clear();
            }
        }

        //press l bumper to reverse intake
        if (gamepad1.left_bumper) {
            if(intakeOutSwitch.flip()) {
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
            if (intakeInSwitch.flip()) {
                if (intakeMode == IntakeMode.IN) {
                    intakeMode = IntakeMode.OFF;
                } else {
                    intakeMode = IntakeMode.IN;
                }
                updateIntake();
            }
        }

        //press b to rotate stone 90 degrees
        if (gamepad2.b) {
            wristToggle.flip();
            if (wristToggle.isEnabled()) {
                wrist.setPosition(0); //todo reestablish wrist
            } else {
                wrist.setPosition(0);
            }
        }

        //press a to grip block
        if (gamepad2.a) {
            gripperToggle.flip();
            activateGripper();
        }

        //press a to grip foundation
        if (gamepad1.a) {
            foundationToggle.flip();
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
            collectionModeToggle.flip();
            if (collectionModeToggle.isEnabled() && intakeMode == IntakeMode.IN && intakeTrigger.isTriggered()) {//if playing delivery, stop intake at trigger.
                intakeMode = IntakeMode.OFF;
                updateIntake();
            }
            telemetry.addData("Collection Mode?", collectionModeToggle.isEnabled());
            telemetry.update();
        }
    }

    private void updateIntake() {
        if(intakeMode == IntakeMode.IN){
            intake.runMotors(INTAKE_POWER);
        } else if (intakeMode == IntakeMode.OFF) {
            intake.stopMotors();
        } else if(intakeMode == IntakeMode.OUT){
            intake.runMotors(-INTAKE_POWER);
        }
    }

    private void activateGripper() {
        if(gripperToggle.isEnabled()){
            gripper.setPosition(1);
        } else {
            gripper.setPosition(0);
        }
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
        slides.stopMotors();
    }

    private class Switch {
        private final static double BUTTON_THRESHOLD = 300; //in millis - time between presses
        protected double lastPress;

        public Switch() {
            lastPress = time.milliseconds();
        }

        public boolean flip() {
            if(time.milliseconds() - lastPress > BUTTON_THRESHOLD) {
                lastPress = time.milliseconds();
                return true;
            }
            return false;
        }
    }

    private class Toggle extends Switch {
        private boolean enabled = false;

        public Toggle() {
            super();
        }

        public boolean flip() {
            if (super.flip()) {
                enabled = !enabled;
                return true;
            }
            return false;
        }

        public boolean isEnabled() {
            return enabled;
        }
    }
}
