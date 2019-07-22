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

package edu.ahs.robotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

@TeleOp(name="Robot Model Test Opmode", group="Linear Opmode")
//@Disabled
public class RobotModelOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {
        FTCUtilities.setHardwareMap(hardwareMap);
        FTCUtilities.setOpMode(this);

        Robot jankBot = initRobot();
        waitForStart();
        runtime.reset();
        jankBot.execute();
        // MecanumChassis chassis = (MecanumChassis)jankBot.getChassis();
        //chassis.turnOnMotor();

    }

    Robot initRobot() { //accessible from JUnit tests
        HashMap<MotorLocations, String> deviceNames = new HashMap<>();
        HashMap<MotorLocations, Boolean> motorFlips = new HashMap<>();
        BotConfig jankBotConfig = new BotConfig();
        //set BotConfig Parameters prior to creation of main Robot object
        jankBotConfig.setChassisType(MecanumChassis.class);
        MotorHashService.init();
        jankBotConfig.setDriveMotors(MotorHashService.MotorTypes.YJ_435);
        jankBotConfig.setWheelDiameter(3.94);// in inches
        jankBotConfig.setDriveGears(1,2);

        deviceNames.put(MotorLocations.FRONTLEFT, "FL");
        deviceNames.put(MotorLocations.FRONTRIGHT, "FR");
        deviceNames.put(MotorLocations.BACKLEFT, "BL");
        deviceNames.put(MotorLocations.BACKRIGHT, "BR");

        motorFlips.put(MotorLocations.FRONTLEFT, false);
        motorFlips.put(MotorLocations.FRONTRIGHT, true);
        motorFlips.put(MotorLocations.BACKLEFT, false);
        motorFlips.put(MotorLocations.BACKRIGHT, true);

        jankBotConfig.setDriveMotorDeviceNames(deviceNames);
        jankBotConfig.setFlippedMotors(motorFlips);


        //Create main Robot Object
        Robot jankBot = new Robot(jankBotConfig);
        Plan gamePlan = new Plan();
        //start constructing PlanElements below
        gamePlan.addToPlan(new ForwardMotion(60, 1, 10000, jankBot.getChassis()));
        jankBot.givePlan(gamePlan);
        //End of Init
        return jankBot;

    }
}
