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

package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.GCodeReader;
import edu.ahs.robotics.util.Logger;


@Autonomous(name = "--- New Auto ---", group = "Linear Opmode")
@Disabled
public class ArdennesNewAutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        FTCUtilities.setOpMode(this);

        Ardennes ardennes = new Ardennes();
        MecanumChassis chassis = ardennes.getChassis();
        //chassis.setPosition();

        Logger logger = new Logger("newAutoData", "pathFollower");

        Path firstMovement = new Path(GCodeReader.openFile("Intake.csv"),12,12,36);
        Path secondMovement = new Path(GCodeReader.openFile("BackwardsToFoundation.csv"),12,12,36);
        Path thirdMovement = new Path(GCodeReader.openFile("backUnderBridge.csv"),12,12,36);

        waitForStart();

        chassis.startOdometrySystem();

        telemetry.addLine("part 1");
        telemetry.update();

        chassis.followPath(firstMovement, true,0.2,0.15);

        telemetry.addLine("part 2");
        telemetry.update();

        chassis.followPath(secondMovement, false, 0.15,0.25);

        telemetry.addLine("part 3");
        telemetry.update();

        chassis.followPath(thirdMovement, true,0.1,0.1);

        logger.stopWriting();
    }
}