package org.firstinspires.ftc.teamcode.live;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.hardware.SerialServo;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.ftc.FTCUtilities;
import edu.ahs.robotics.util.MotorHashService;

@Autonomous(name = "Drive Under Bridge", group = "Linear Opmode")
@Disabled
public class DriveUnderBridgeAuto extends LinearOpMode {
    private Ardennes ardennes;
    private MecanumChassis chassis;
    private SerialServo intakeServo;
    private SerialServo captone;

    @Override
    public void runOpMode() {
        FTCUtilities.setOpMode(this);
        MotorHashService.init();
        ardennes = new Ardennes();
        chassis = ardennes.getChassis();
        captone = ardennes.getCapstone();

        waitForStart();
        captone.setPosition(-.5);
        chassis.driveStraight(4, .93);
    }

}
