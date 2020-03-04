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
 * ARE DISCLAIMED. INSLOW NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER INSLOW CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING INSLOW ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package edu.ahs.robotics.util.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import edu.ahs.robotics.hardware.sensors.Odometer;
import edu.ahs.robotics.hardware.sensors.OdometerImpl;
import edu.ahs.robotics.util.ftc.FTCUtilities;


/**
 * OpMode that returns displacement values to check if wheels are correctly tuned. Also works to check ticks per rotation and rotation direction
 * <b>Independent of BotModel, needs to be tuned before use</b>
 * Remember that motor flips in botmodel may also affect your encoder directions.
 * @author Alex Appleby
 */
@TeleOp(name="Odometer tuning check OpMode", group="Iterative OpMode")
@Disabled
public class WheelTuningCheckOpMode extends OpMode
{
    private Odometer leftOdom,rightOdom,backOdom;
    private DcMotor br;

    @Override
    public void init() {
        FTCUtilities.setOpMode(this);

        br = hardwareMap.get(DcMotor.class, "BR");
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        leftOdom = new OdometerImpl("intakeL",2.3596, true, 1440); // remember to update these
        rightOdom = new OdometerImpl("intakeR",2.3617, true, 1440); // tune all before use
        backOdom = new OdometerImpl("BR",2.387, true, 4000);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        if(gamepad1.a){
            if(br.getDirection() == DcMotorSimple.Direction.REVERSE) {
                br.setDirection(DcMotorSimple.Direction.FORWARD);
            } else {
                br.setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }

        telemetry.addData("back right direction", br.getDirection().toString());

        telemetry.addData("left reading", leftOdom.getDistance());
        telemetry.addData("right reading", rightOdom.getDistance());
        telemetry.addData("back reading", backOdom.getDistance());
        telemetry.update();
    }
    @Override
    public void stop() {

    }

}
