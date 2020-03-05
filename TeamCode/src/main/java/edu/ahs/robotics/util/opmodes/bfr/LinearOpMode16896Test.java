package edu.ahs.robotics.util.opmodes.bfr;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import edu.ahs.robotics.util.opmodes.bfr.LinearOpMode16896;

@Autonomous(name = "Test 16896 LinearOpMode", group = "Linear Opmode")
@Disabled
public class LinearOpMode16896Test extends LinearOpMode16896 {


    @Override
    protected void initialize() {
        telemetry.addLine("in init");
        telemetry.update();
    }

    @Override
    protected void runProgram() {

        telemetry.addLine("in start");
        telemetry.update();

        sleep(1000);

        throw new Error("that one error");
    }

}
