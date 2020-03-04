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

    @Override
    public void runOpMode(){
        FTCUtilities.setOpMode(this);

        Ardennes ardennes = new Ardennes();
        Slides slides = ardennes.getSlides();
        slides.setGamepad(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            slides.gamepadControl();
        }
    }
}
