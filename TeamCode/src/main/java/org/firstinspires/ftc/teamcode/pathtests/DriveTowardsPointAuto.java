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

import edu.ahs.robotics.control.MotionConfig;
import edu.ahs.robotics.control.Point;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.loggers.DataLogger;
import edu.ahs.robotics.util.FTCUtilities;


@Autonomous(name = "Drive Towards Point", group = "Linear Opmode")
//@Disabled
public class DriveTowardsPointAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        FTCUtilities.setOpMode(this);

        Ardennes ardennes = new Ardennes();
        MecanumChassis chassis = ardennes.getChassis();
        chassis.setPosition(0,0,Math.PI);

        Point targetPoint = new Point(60, 12);

        DataLogger logger = new DataLogger("driveTowardPointData", "driveTowardPointData");

        waitForStart(); // -----------------------------

        chassis.startOdometrySystem();
        logger.startWriting();

        double distanceToPoint = targetPoint.distanceTo(chassis.getState().position);

        while (distanceToPoint > 3 && opModeIsActive()){
            OdometrySystem.State state = chassis.getState();

            distanceToPoint = targetPoint.distanceTo(state.position);

            MotionConfig motionConfig = new MotionConfig();

            chassis.driveTowardsPoint(targetPoint,0.8, motionConfig);

            logger.append("x", String.valueOf(state.position.x));
            logger.append("y", String.valueOf(state.position.y));
            logger.append("heading", String.valueOf(state.position.heading));
            logger.writeLine();
        }

        chassis.stopMotors();
        logger.stopWriting();
    }
}
