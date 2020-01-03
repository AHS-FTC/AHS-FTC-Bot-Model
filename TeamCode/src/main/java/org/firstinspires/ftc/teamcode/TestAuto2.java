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

import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.sensors.ArdennesSkyStoneDetector;
import edu.ahs.robotics.hardware.sensors.Odometer;
import edu.ahs.robotics.hardware.sensors.OdometerImpl;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.MotorHashService;
import edu.ahs.robotics.util.Tuner;


@Autonomous(name = "Test Auto 2", group = "Linear Opmode")
//@Disabled
public class TestAuto2 extends LinearOpMode {

    Odometer left, right, back;

    private Ardennes ardennes;

//    private ElapsedTime runtime = new ElapsedTime();
//    private Tuner tuner;
//    private ArdennesSkyStoneDetector detector;
//    private TriggerDistanceSensor intakeTrigger;

    @Override
    public void runOpMode() {

        FTCUtilities.setOpMode(this);
        MotorHashService.init();
//        tuner = new Tuner();
//        FTCUtilities.setParameterLookup(tuner);
//        detector = new ArdennesSkyStoneDetector(false, true);
//        Intake intake = ardennes.getIntake();

        double distanceFromCenter1;
        double distanceFromCenter2;
        double backWheelInchesPerDegree;
        double deltaDegrees = 360.0;

        left = new OdometerImpl("intakeL", 2.3596, false, 1440); // tune these. Make sure odometers rotate forward
        right = new OdometerImpl("intakeR", 2.3617, true, 1440); // tune these
        back = new OdometerImpl("BR", 2.387, false, 4000); // tune these

        left.reset();
        right.reset();
        back.reset();

        ardennes = new Ardennes();
        MecanumChassis chassis = ardennes.getChassis();

        waitForStart();

        chassis.pivot(360.0,.75, .6, .55, 15000);

            while (true) {

                distanceFromCenter1 = left.getDistance()/Math.PI;
                distanceFromCenter2 = right.getDistance()/Math.PI;

                backWheelInchesPerDegree = back.getDistance() / deltaDegrees;

                telemetry.addData("distance from center 1", distanceFromCenter1);
                telemetry.addData("distance from center 2", distanceFromCenter2);
                telemetry.addData("back wheel inches per degree", backWheelInchesPerDegree);
                telemetry.addData("left - ins", left.getDistance());
                telemetry.addData("right - ins", right.getDistance());
                telemetry.addData("back - ins", back.getDistance());
                telemetry.update();
            }
    }
}
