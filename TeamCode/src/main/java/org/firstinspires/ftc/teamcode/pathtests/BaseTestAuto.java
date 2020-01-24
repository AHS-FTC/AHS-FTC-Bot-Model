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
    private boolean forwards;
    private double leftInitialPower, rightInitialPower;

    public BaseTestAuto(ArrayList<Point> points, boolean forwards, double leftInitialPower, double rightInitialPower){
        this.forwards = forwards;
        MotorHashService.init();
        ardennes = new Ardennes();
        chassis = ardennes.getChassis();
        this.leftInitialPower = leftInitialPower;
        this.rightInitialPower = rightInitialPower;
        path = new Path(points, 12, 4, 36);
        chassis.startOdometrySystem();
    }

    public void afterStart(){
        chassis.followPath(path, forwards, leftInitialPower, rightInitialPower);

        FTCUtilities.sleep(1000);
        chassis.stopOdometrySystem();
        Logger.stopLoggers();
    }
}
