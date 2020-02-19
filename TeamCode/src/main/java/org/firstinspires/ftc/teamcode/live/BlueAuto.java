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

package org.firstinspires.ftc.teamcode.live;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.ahs.robotics.hardware.sensors.ArdennesSkyStoneDetector;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.ftc.FTCUtilities;


@Autonomous(name = "-- AutoBlue --", group = "Linear Opmode")
//@Disabled
public class BlueAuto extends LinearOpMode {

    ArdennesSkyStoneDetector detector;
    Ardennes ardennes;

    @Override
    public void runOpMode() {
        FTCUtilities.setOpMode(this);
/*
        ArdennesSkyStoneDetector detector = new ArdennesSkyStoneDetector(false, false);
        BaseAuto auto = new BaseAuto(true);

        List<List<Point>> route36 = GCodeReader.openFile("foo");
        List<List<Point>> route25 = GCodeReader.openFile("foo");
        List<List<Point>> route14 = GCodeReader.openFile("foo");

        Path quarry36 = new Path(route36.get(0), 12,12,22, false);
        Path toFoundation36 = new Path(route36.get(1), 8,4,35, 9, 2, 1, false); //32
        Path quarry236 = new Path(route36.get(2), 12, 12, 20,6,2,1, false);
        Path foundation236 = new Path(route36.get(3), 12, 12, 44,9,2,1, false);
        Path quarry336 = new Path(route36.get(4), 12, 12, 30,9,2,1, false);
        Path foundation336 = new Path(route36.get(5), 12, 12, 40,9,2,1, false);

        Path quarry25 = new Path(route25.get(0), 12,12,22, false);
        Path toFoundation25 = new Path(route25.get(1), 8,4,35, 9, 2, 1, false); //32
        Path quarry225 = new Path(route25.get(2), 12, 12, 36,6,2,1, false);
        Path foundation225 = new Path(route25.get(3), 12, 12, 44,9,2,1, false);
        Path quarry325 = new Path(route25.get(4), 12, 12, 30,9,2,1, false);
        Path foundation325 = new Path(route25.get(5), 12, 12, 40,9,2,1, false);

        Path quarry14 = new Path(route14.get(0), 12,12,22, 18, 2,1, false);
        Path toFoundation14 = new Path(route14.get(1), 8,4,35, 9, 2, 1, false); //32
        Path quarry214 = new Path(route14.get(2), 12, 12, 36,6,2,1, false);
        Path foundation214 = new Path(route14.get(3), 12, 12, 44,9,2,1, false);
        Path quarry314 = new Path(route14.get(4), 12, 12, 44,9,2,1, false);
        Path foundation314 = new Path(route14.get(5), 12, 12, 44,9,2,1, false);

        Path quarry;
        Path toFoundation;
        Path quarry2;
        Path foundation2;
        Path quarry3;
        Path foundation3;

        waitForStart(); //-----------------------------
        ArdennesSkyStoneDetector.SkyStoneConfigurations stoneConfiguration = detector.look();

        if (stoneConfiguration == ArdennesSkyStoneDetector.SkyStoneConfigurations.ONE_FOUR) {
            quarry = quarry14;
            toFoundation = toFoundation14;
            quarry2 = quarry214;
            foundation2 = foundation214;
            quarry3 = quarry314;
            foundation3 = foundation314;

        } else if (stoneConfiguration == ArdennesSkyStoneDetector.SkyStoneConfigurations.TWO_FIVE) {
            quarry = quarry25;
            toFoundation = toFoundation25;
            quarry2 = quarry225;
            foundation2 = foundation225;
            quarry3 = quarry325;
            foundation3 = foundation325;

        } else {
            quarry = quarry36;
            toFoundation = toFoundation36;
            quarry2 = quarry236;
            foundation2 = foundation236;
            quarry3 = quarry336;
            foundation3 = foundation336;

        }

        auto.setPaths(quarry, toFoundation, null, quarry2, foundation2, quarry3, foundation3);

        auto.start();
*/
    }

}
