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

package org.firstinspires.ftc.teamcode.pathtests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import edu.ahs.robotics.control.MotionConfig;
import edu.ahs.robotics.control.Point;
import edu.ahs.robotics.util.GCodeReader;
import edu.ahs.robotics.util.ftc.FTCUtilities;
import edu.ahs.robotics.util.loggers.DataLogger;
import edu.ahs.robotics.util.loggers.Logger;
import edu.ahs.robotics.util.opmodes.bfr.LinearOpMode16896;


@Autonomous(name = "Left Curve Auto Backwards", group = "Linear Opmode")
//@Disabled
public class LeftCurveAutoBackwards extends LinearOpMode16896 {

    private ElapsedTime runtime = new ElapsedTime();
    Logger logger;
    BaseTestAuto base;

    @Override
    protected void initialize() {
        FTCUtilities.setOpMode(this);

        logger = new DataLogger("pathDataCurveLBackwards", "partialPursuit");

        List<List<Point>> points = GCodeReader.openFile("1001.csv");

        MotionConfig motionConfig = new MotionConfig();
        motionConfig.idealHeading = Math.PI;

        base = new BaseTestAuto(points.get(0), .3, .3, new double[][]{{6, .4}, {12, .5}, {18, .4}}, motionConfig);
    }

    @Override
    protected void runProgram() {
        base.afterStart();
        logger.stopWriting();
    }
}
