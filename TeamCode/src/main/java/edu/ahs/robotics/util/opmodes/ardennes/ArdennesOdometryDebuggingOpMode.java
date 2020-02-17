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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import edu.ahs.robotics.control.Position;
import edu.ahs.robotics.control.Velocity;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.loggers.DataLogger;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.loggers.Logger;

/**
 * Test OpMode for logging and debugging the Ardennes OdometrySystemImpl.
 * @author Alex Appleby
 */
@TeleOp(name="Ardennes Odometery Debugging OpMode", group="Iterative OpMode")
//@Disabled
public class ArdennesOdometryDebuggingOpMode extends OpMode
{
    private Position position;
    private Velocity velocity;
    private double startTime = 0;
    private DataLogger logger;
    private Ardennes ardennes;
    private MecanumChassis chassis;

    private DcMotor fl,fr,bl,br;

    private double lastTime;


    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, "FL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FTCUtilities.setOpMode(this);
        ardennes = new Ardennes();
        chassis = ardennes.getChassis();
        logger = new DataLogger("odometryDebuggingOpMode", "odometryDebuggingOpMode");
        logger.startWriting();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        lastTime = FTCUtilities.getCurrentTimeMillis();
        startTime = FTCUtilities.getCurrentTimeMillis();
        chassis.startOdometrySystem();
    }

    @Override
    public void loop() {

        if(gamepad1.b){
            fl.setPower(-0.5);
            fr.setPower(0.5);
            bl.setPower(0.5);
            br.setPower(-0.5);
        } else {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }

        double currentTime = FTCUtilities.getCurrentTimeMillis();

        OdometrySystem.State state = chassis.getState();

        position = state.position;

        FTCUtilities.addData("x", String.valueOf(position.x));
        FTCUtilities.addData("y", String.valueOf(position.y));
        FTCUtilities.addData("heading", String.valueOf(position.getHeadingInDegrees()));

        logger.append("x", String.valueOf(position.x));
        logger.append("y", String.valueOf(position.y));
        logger.append("heading", String.valueOf(position.getHeadingInDegrees()));
        logger.writeLine();

        lastTime = currentTime;
    }
    @Override
    public void stop() {
        Logger.stopLoggers();
        chassis.stopOdometrySystem();
    }

}
