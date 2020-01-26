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

package org.firstinspires.ftc.teamcode.pathtests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.Point;
import edu.ahs.robotics.hardware.Chassis;
import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.GCodeReader;
import edu.ahs.robotics.util.Logger;
import edu.ahs.robotics.util.MotorHashService;


@Autonomous(name = "-- Partial Pursuit Test Auto --", group = "Linear Opmode")
//@Disabled
public class PartialPursuitTestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        FTCUtilities.setOpMode(this);
        MotorHashService.init();

        Ardennes ardennes = new Ardennes();

        MecanumChassis chassis = ardennes.getChassis();
        Intake intake = ardennes.getIntake();

        Logger logger = new Logger("pathDataCurveLReverse", "partialPursuit");

        Path quarry = new Path(GCodeReader.openFile("3-6-1_quarry.csv"), 8,6,20);
        Path toFoundation = new Path(GCodeReader.openFile("3-6-2_foundation.csv"), 8,4,32);

        chassis.setPosition(63,-40, Math.PI);
        chassis.startOdometrySystem();

        waitForStart(); //-----------------------------

        intake.startIntakeWaitForBlock(ardennes.getGripperTrigger());

        chassis.followPath(quarry, 12, 0);
        chassis.followPath(toFoundation, 12, Math.PI);
        chassis.globalPointTurn((2*Math.PI), 0.3, 2500);

        intake.stopMotors();

        chassis.stopMotors();
        logger.stopWriting();
    }
}
