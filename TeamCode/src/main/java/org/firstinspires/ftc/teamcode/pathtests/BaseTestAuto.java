package org.firstinspires.ftc.teamcode.pathtests;

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

    public BaseTestAuto(ArrayList<Point> points){
        MotorHashService.init();
        ardennes = new Ardennes();
        chassis = ardennes.getChassis();
        path = new Path(points, 12, 8, 28);//was 36
        chassis.startOdometrySystem();
    }


    public void afterStart(){
        chassis.followPath(path, 12, 0);
    }
}
