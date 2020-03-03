package edu.ahs.robotics.util.opmodes.ardennes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ahs.robotics.hardware.Slides;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.ftc.FTCUtilities;

@TeleOp(name = "Slides PID Test", group = "Iterative OpMode")
//@Disabled
public class TestSlidesPID extends LinearOpMode {
    private static final double CONSTANT_MULTIPLIER = 0.001;

    @Override
    public void runOpMode(){
        FTCUtilities.setOpMode(this);

        Ardennes ardennes = new Ardennes();
        Slides slides = ardennes.getSlides();
        slides.setGamepad(gamepad1);

        double upCorrection = 0.25 * CONSTANT_MULTIPLIER, downCorrection = 0.13 * CONSTANT_MULTIPLIER;

        waitForStart();

        slides.runAtPower(.7);

        while (opModeIsActive()) {
            upCorrection +=    (gamepad1.left_stick_y * 0.001 * CONSTANT_MULTIPLIER);
            downCorrection += (gamepad1.right_stick_y * 0.001 * CONSTANT_MULTIPLIER);

            slides.tuningPIDControl(upCorrection, downCorrection);

            telemetry.addData("slide height", slides.getCurrentPosition());
            telemetry.addData("up correction", upCorrection / CONSTANT_MULTIPLIER);
            telemetry.addData("down correction", downCorrection / CONSTANT_MULTIPLIER);
            telemetry.update();
        }
    }
}
