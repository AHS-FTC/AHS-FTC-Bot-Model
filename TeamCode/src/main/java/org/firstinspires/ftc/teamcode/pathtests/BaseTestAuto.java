package org.firstinspires.ftc.teamcode.pathtests;

import java.util.List;

import edu.ahs.robotics.control.MotionConfig;
import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.Point;
import edu.ahs.robotics.control.obm.NullCommand;
import edu.ahs.robotics.control.obm.OBMCommand;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.MotorHashService;

public class BaseTestAuto {
    private Ardennes ardennes;
    private Path path;
    private MecanumChassis chassis;
    private OBMCommand nullCommand = new NullCommand();

    public BaseTestAuto(List<Point> points){
        MotorHashService.init();
        ardennes = new Ardennes();
        chassis = ardennes.getChassis();
        path = new Path(points, false, .5, new double[][]{});//was 36
        chassis.startOdometrySystem();
    }


    public void afterStart(){
        MotionConfig pathConfig = new MotionConfig();

        chassis.followPath(path, pathConfig);
        chassis.stopMotors();
        FTCUtilities.sleep(2000);
        chassis.stopOdometrySystem();
    }
}
