package org.firstinspires.ftc.teamcode.pathtests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

import edu.ahs.robotics.control.MotionConfig;
import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.Point;
import edu.ahs.robotics.control.obm.FoundationFinder;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.MotorHashService;
import edu.ahs.robotics.util.ftc.ErrorStealer;
import edu.ahs.robotics.util.ftc.FTCUtilities;
import edu.ahs.robotics.util.loggers.DataLogger;
import edu.ahs.robotics.util.loggers.Logger;

@Autonomous(name = "Foundation Look Auto", group = "Linear Opmode")
//@Disabled
public class FoundationLookTest extends LinearOpMode {

    Ardennes ardennes;
    private MecanumChassis chassis;

    @Override
    public void runOpMode() {
        FTCUtilities.setOpMode(this);

        Logger logger = new DataLogger("pathDataFoundation", "partialPursuit");

        MotorHashService.init();
        ardennes = new Ardennes();
        chassis = ardennes.getChassis();
        FoundationFinder foundationFinder = new FoundationFinder(ardennes, 10, true);

        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0,0));
        points.add(new Point(0,72));

        Path toFoundation = new Path(points, false, .3, .3, new double[][]{{30,.3}, {50,.3}});//was 36
        MotionConfig toFoundationConfig = new MotionConfig();
        toFoundationConfig.addOBMCommand(foundationFinder);
        toFoundationConfig.idealHeading = Math.PI/2;

        waitForStart();
        try {
            chassis.startOdometrySystem();
            chassis.followPath(toFoundation, toFoundationConfig);
            chassis.stopMotors();
            chassis.stopOdometrySystem();
            logger.stopWriting();
        } catch (Throwable t){
            ErrorStealer.stealError(t);
        }


    }

}
