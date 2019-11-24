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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.ahs.robotics.abbreviatedmodel.Ardennes;
import edu.ahs.robotics.autocommands.Plan;
import edu.ahs.robotics.autocommands.autopaths.ForwardMotion;
import edu.ahs.robotics.autocommands.autopaths.OdometryForwardMotion;
import edu.ahs.robotics.autocommands.autopaths.OdometryPointTurn;
import edu.ahs.robotics.autocommands.autopaths.Sleep;
import edu.ahs.robotics.autocommands.obmcommands.IntakeCommand;
import edu.ahs.robotics.autocommands.obmcommands.IntakeCommandWithTrigger;
import edu.ahs.robotics.botfactories.ArdennesFactory;
import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.hardware.Robot;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.MotorHashService;


//@TeleOp(name="Ardennes Auto", group="Linear Opmode")
@Autonomous(name = "Ardennes Auto 2 Electric Boogaloo", group = "Linear Opmode")
//@Disabled
public class ArdennesAuto2ElectricBoogaloo extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Ardennes ardennes;
    private Plan plan;


    @Override
    public void runOpMode() {
        FTCUtilities.setOpMode(this);
        FTCUtilities.setHardwareMap(hardwareMap);
        MotorHashService.init();

        plan = new Plan();

        ardennes = new Ardennes();
        plan.addToPlan(new IntakeCommandWithTrigger(ardennes.getIntake(), ardennes.getIntakeTrigger()));
        ardennes.givePlan(plan);

        waitForStart();

        ardennes.executePlan();
    }

}
