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

import edu.ahs.robotics.util.opmodes.SimpleTeleOp;


/**
 * Test OpMode for tuning the true diameter of the odometer wheels on Ardennes. The 60mm REV wheels have bad tolerances.
 * This OpMode doesn't utilize the Ardennes class structure and instead directly accesses the encoders via the hardwaremap.
 * You may have to look over this code to make sure it's accurate before using/reusing it.
 * </br>
 * Using this class does require some math. Take the total distance you push your robot and divide it by the amount of rotations to figure out the circumference of the wheel.
 * Knowing the circumference calculate diameter by dividing by pi.
 * The diameter of each wheel should come out to be close to 60mm or 2.36 inches (if you're using the Ardennes rev wheels).
 * @author Alex Appleby
 */
@TeleOp(name="Ardennes Odometery Wheel Tuner", group="Iterative OpMode")
//@Disabled
public class ArdennesWheelTuningOpMode extends OpMode
{
    private DcMotor left, right, back;
    private static final double TICKS_PER_ROTATION = 1440;
    private SimpleTeleOp tele;

    @Override
    public void init() {
        left = hardwareMap.get(DcMotor.class,"intakeL");
        right = hardwareMap.get(DcMotor.class,"intakeR");
        back = hardwareMap.get(DcMotor.class,"BR");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tele = new SimpleTeleOp();
        tele.hardwareMap = hardwareMap;
        tele.init();
        tele.gamepad1 = gamepad1;
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        tele.loop();
        telemetry.addData("Left Rotations", (left.getCurrentPosition()/TICKS_PER_ROTATION));
        telemetry.addData("Right Rotations", (right.getCurrentPosition()/TICKS_PER_ROTATION));
        telemetry.addData("Back Rotations", (back.getCurrentPosition()/TICKS_PER_ROTATION));
        telemetry.addData("back reading", back.getCurrentPosition());
        telemetry.update();
    }
    @Override
    public void stop() {

    }

}
