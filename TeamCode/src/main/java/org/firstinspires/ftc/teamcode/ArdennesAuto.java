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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.hardware.Slides;
import edu.ahs.robotics.hardware.sensors.ArdennesSkyStoneDetector;
import edu.ahs.robotics.hardware.sensors.LimitSwitch;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.MotorHashService;


@Autonomous(name = "Ardennes Auto", group = "Linear Opmode")
//@Disabled
public class ArdennesAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Ardennes ardennes;


    @Override
    public void runOpMode() {

        FTCUtilities.setOpMode(this);
        MotorHashService.init();
        ardennes = new Ardennes();
        Intake intake = ardennes.getIntake();
        MecanumChassis chassis = ardennes.getChassis();
        Slides slides = ardennes.getSlides();
        SerialServo foundationServoLeft = ardennes.getLeftFoundation();
        SerialServo foundationServoRight = ardennes.getRightFoundation();
        SerialServo gripper = ardennes.getGripper();
        SerialServo yslide = ardennes.getySlide();
        TriggerDistanceSensor gripperTrigger = ardennes.getGripperTrigger();
        slides.resetEncoders();
        gripper.setPosition(0);
        telemetry.addLine("Init Finished");
        telemetry.update();

        waitForStart();

        ArdennesSkyStoneDetector.SkyStoneConfigurations stoneConfiguration = ardennes.runDetector();
        if (ArdennesSkyStoneDetector.SkyStoneConfigurations.ONE_FOUR == stoneConfiguration){
            leftPlan(intake, chassis, foundationServoLeft, foundationServoRight, slides);
        } else if (ArdennesSkyStoneDetector.SkyStoneConfigurations.TWO_FIVE == stoneConfiguration){
            middlePlan(intake, chassis, foundationServoLeft, foundationServoRight, slides);
        } else rightPlan(intake, chassis, foundationServoLeft, foundationServoRight, slides);

    }

    private void leftPlan(Intake intake, MecanumChassis chassis, SerialServo foundationServoLeft, SerialServo foundationServoRight, Slides slides) {
        chassis.driveStraight(500, 1);
        chassis.pivot(-30,.4);
        intake.startIntakeWaitForBlock(ardennes.getIntakeTrigger());
        chassis.driveStraight(450,.3);

    }

    private void middlePlan(Intake intake, MecanumChassis chassis, SerialServo foundationServoLeft, SerialServo foundationServoRight, Slides slides) {
        chassis.pivot(10, .4);
        chassis.driveStraight(600,1);
        chassis.pivot(-40, .4);
        intake.startIntakeWaitForBlock(ardennes.getIntakeTrigger());
        chassis.driveStraight(450, .3);
        chassis.driveStraight(-450, 1);
        chassis.pivot(-57,.4);
        chassis.driveStraight(-1700,.8);
        chassis.pivot(-87,.5);
        chassis.driveStraight(-200, .4);
        foundationServoLeft.setPosition(0);
        foundationServoRight.setPosition(1);
        sleep(1500);
        chassis.driveStraight(700, 1);
        chassis.pivot(93,.5);
        foundationServoLeft.setPosition(1);
        foundationServoRight.setPosition(0);
        sleep(700);
        sleep(1000);



    }

    private void rightPlan(Intake intake, MecanumChassis chassis, SerialServo foundationServoLeft, SerialServo foundationServoRight, Slides slides) {
        chassis.pivot(-10, .4);
        chassis.driveStraight(550,1);
        chassis.pivot(30,.5);
        intake.startIntakeWaitForBlock(ardennes.getIntakeTrigger());
        chassis.driveStraight(300,.3);

    }


}
