package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.MotorHashService;

@Autonomous(name = "Drive Under Bridge", group = "Linear Opmode")

public class DriveUnderBridgeAuto extends LinearOpMode {
    private Ardennes ardennes;
    private MecanumChassis chassis;
    private SerialServo intakeServo;

    @Override
    public void runOpMode() {
        FTCUtilities.setOpMode(this);
        MotorHashService.init();
        ardennes = new Ardennes();
        chassis = ardennes.getChassis();

        waitForStart();
        sleep(20000);
        chassis.driveStraight(300, .93);
    }

}
