package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.Point;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.Logger;
import edu.ahs.robotics.util.MotorHashService;

public class BaseTestAuto {
    private Ardennes ardennes;
    private Path path;
    private MecanumChassis chassis;
    private boolean forwards;

    public BaseTestAuto(ArrayList<Point> points, boolean forwards){
        MotorHashService.init();
        ardennes = new Ardennes();
        chassis = ardennes.getChassis();
        path = new Path(points, 12, 4, 36);
        chassis.startOdometrySystem();
        this.forwards = forwards;
    }

    public void afterStart(){
        chassis.followPath(path, forwards);

        FTCUtilities.sleep(250);
        chassis.stopOdometrySystem();
        Logger.stopLoggers();
    }
}
