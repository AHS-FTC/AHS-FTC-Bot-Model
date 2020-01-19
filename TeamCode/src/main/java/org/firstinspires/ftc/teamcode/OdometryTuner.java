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


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import java.util.ArrayList;

import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.sensors.IMU;
import edu.ahs.robotics.hardware.sensors.Odometer;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.Logger;


@Autonomous(name = "Odometry Tuner", group = "Linear Opmode")
@Disabled
public class OdometryTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        FTCUtilities.setOpMode(this);

        Ardennes ardennes = new Ardennes();
        MecanumChassis chassis = ardennes.getChassis();
        IMU imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));

        ArrayList<Odometer> odometers = chassis.getOdometers();
        Odometer left = odometers.get(0);
        Odometer right = odometers.get(1);
        Odometer back = odometers.get(2);

        waitForStart();
        double initialIMU = imu.getHeading();
        double deltaHeading = 0;

        while (deltaHeading < 90){
            telemetry.addData("heading", imu.getHeading());
            telemetry.update();

            chassis.drive3Axis(0,0,0.3);
            deltaHeading = imu.getHeading() - initialIMU;
        }
        chassis.stopMotors();

        double averageOdomMovement = (Math.abs(left.getDistance()) + Math.abs(right.getDistance()))/2;
        double rotationProportion = deltaHeading / 360;
        double distanceBetweenWheels = averageOdomMovement / (Math.PI * rotationProportion);

        double yInchesPerDegree =  back.getDistance() / deltaHeading;

        telemetry.addData("delta heading" , deltaHeading);
        telemetry.addData("averageOdomMovement", averageOdomMovement);
        telemetry.addData("distance between wheels", distanceBetweenWheels);
        telemetry.addData("y inches per degree", yInchesPerDegree);
        telemetry.update();
        Logger.stopLoggers();
        sleep(10000);
    }
}
