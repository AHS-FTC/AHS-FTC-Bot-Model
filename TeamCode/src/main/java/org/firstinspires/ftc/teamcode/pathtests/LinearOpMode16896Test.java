package org.firstinspires.ftc.teamcode.pathtests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ahs.robotics.util.opmodes.LinearOpMode16896;

@Autonomous(name = "Test 16896 LinearOpMode", group = "Linear Opmode")
@Disabled
public class LinearOpMode16896Test extends LinearOpMode16896 {

    @Override
    protected void runProgram() {
        telemetry.addLine("in init");
        telemetry.update();

        waitForStart();

        throw new Error("that one error");
    }
}
