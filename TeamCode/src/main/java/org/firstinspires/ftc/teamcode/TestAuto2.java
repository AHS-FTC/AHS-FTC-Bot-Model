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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autos.PartialPursuitAuto;

import java.util.ArrayList;

import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.Point;
import edu.ahs.robotics.hardware.ChassisMotors;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.sensors.ArdennesSkyStoneDetector;
import edu.ahs.robotics.hardware.sensors.Odometer;
import edu.ahs.robotics.hardware.sensors.OdometerImpl;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.GCodeReader;
import edu.ahs.robotics.util.MotorHashService;
import edu.ahs.robotics.util.Tuner;


@Autonomous(name = "Test Auto 2", group = "Linear Opmode")
//@Disabled
public class TestAuto2 extends LinearOpMode {

    Odometer left, right, back;

    private Ardennes ardennes;
    private MecanumChassis chassis;

//    private ElapsedTime runtime = new ElapsedTime();
//    private Tuner tuner;
//    private ArdennesSkyStoneDetector detector;
//    private TriggerDistanceSensor intakeTrigger;

    @Override
    public void runOpMode() {

        FTCUtilities.setOpMode(this);
        MotorHashService.init();
        ardennes = new Ardennes();
        chassis = ardennes.getChassis();

//        tuner = new Tuner();
//        FTCUtilities.setParameterLookup(tuner);
//        detector = new ArdennesSkyStoneDetector(false, true);
//        Intake intake = ardennes.getIntake();

        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(-1,-1));
        points.add(new Point(-3,-3));
        points.add(new Point(-3,-6));
        Path path = new Path(points, 12,12,12,false);

        chassis.followPath(path, 12, 0, null, 10000,0);

    }
}
