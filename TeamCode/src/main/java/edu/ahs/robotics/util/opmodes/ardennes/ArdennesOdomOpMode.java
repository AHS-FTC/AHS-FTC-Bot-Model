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

package edu.ahs.robotics.util.opmodes.ardennes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import edu.ahs.robotics.control.Position;
import edu.ahs.robotics.control.Velocity;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.sensors.IMU;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.loggers.DataLogger;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.loggers.Logger;

/**
 * Test OpMode for logging and debugging the Ardennes OdometrySystemImpl.
 * @author Alex Appleby
 */
@TeleOp(name="Ardennes Odometry OpMode", group="Iterative OpMode")
//@Disabled
public class ArdennesOdomOpMode extends OpMode
{
    //private Ardennes ardennes;
    private Position position;
    private Velocity velocity;
    private Logger logger;
    private Ardennes ardennes;
    private MecanumChassis chassis;
    private IMU imu;


    private double lastTime;

    //private OpMode teleOp;

    @Override
    public void init() {
        FTCUtilities.setOpMode(this);
        ardennes = new Ardennes();
        logger = new DataLogger("ardennesOdomOpMode","ardennesOdomOpMode");

        //teleOp = new SimpleTeleOp();
        //teleOp.hardwareMap = hardwareMap;
        //teleOp.gamepad1 = gamepad1;
        //teleOp.init();
        chassis = ardennes.getChassis();
        chassis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        lastTime = FTCUtilities.getCurrentTimeMillis();
        logger.startWriting();
        chassis.startOdometrySystem();
        chassis.setPosition(0,0,0);
    }

    @Override
    public void loop() {
        //teleOp.loop();
        chassis.drive3Axis(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        double currentTime = FTCUtilities.getCurrentTimeMillis();
        double imuHeading = imu.getHeading();

        OdometrySystem.State state = chassis.getState();

        position = state.position;
        velocity = state.velocity;

        telemetry.addData("x -ins", position.x);
        telemetry.addData("y -ins", position.y);
        telemetry.addData("dySum", state.dySum);
        //telemetry.addData("x^2 + y^2 sqrt", Math.sqrt(position.x * position.x + position.y * position.y));

//        telemetry.addData("heading -deg", Math.toDegrees(position.heading));
//        telemetry.addData("imu heading -deg", imuHeading);
//        telemetry.addData("speed -in/s", velocity.speed());
//        telemetry.addData("dir of travel -deg", Math.toDegrees(velocity.direction()));
//        telemetry.addData("delta time -millis", currentTime - lastTime);
        telemetry.update();
//
//        logger.append("x", String.valueOf(position.x));
//        logger.append("y", String.valueOf(position.y));
//        logger.append("heading", String.valueOf(position.getHeadingInDegrees()));
//        logger.append("speed", String.valueOf(velocity.speed()));
//        logger.append("acceleration", String.valueOf(state.acceleration));
//        logger.append("radius", String.valueOf(state.travelRadius));
//
//        logger.append("imu heading", String.valueOf(imuHeading));
//        logger.writeLine();

        lastTime = currentTime;
    }
    @Override
    public void stop() {
        //teleOp.stop();
        Logger.stopLoggers();
        chassis.stopOdometrySystem();
    }

}
