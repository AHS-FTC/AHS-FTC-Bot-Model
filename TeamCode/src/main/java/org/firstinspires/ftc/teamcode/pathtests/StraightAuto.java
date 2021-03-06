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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

import edu.ahs.robotics.control.Point;
import edu.ahs.robotics.util.ftc.ErrorStealer;
import edu.ahs.robotics.util.loggers.DataLogger;
import edu.ahs.robotics.util.ftc.FTCUtilities;
import edu.ahs.robotics.util.loggers.Logger;


@Autonomous(name = "Straight Auto", group = "Linear Opmode")
@Disabled
public class StraightAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        FTCUtilities.setOpMode(this);

        Logger logger = new DataLogger("pathDataStraight", "partialPursuit");

        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0,0));
        points.add(new Point(72,0));

        BaseTestAuto base = new BaseTestAuto(points,.3, 0, new double[][]{{24, .4}, {48, .4}});

        waitForStart();


        try {
            logger.startWriting();
            base.afterStart();
        } catch (Throwable t){
            ErrorStealer.stealError(t);
        }

        logger.stopWriting();
    }
}
