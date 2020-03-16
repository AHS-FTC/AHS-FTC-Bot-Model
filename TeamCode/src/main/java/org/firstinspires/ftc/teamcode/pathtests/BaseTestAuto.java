package org.firstinspires.ftc.teamcode.pathtests;

import java.util.List;

import edu.ahs.robotics.control.MotionConfig;
import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.Point;
import edu.ahs.robotics.control.obm.NullCommand;
import edu.ahs.robotics.control.obm.OBMCommand;
import edu.ahs.robotics.hardware.MecanumChassis;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.ftc.FTCUtilities;
import edu.ahs.robotics.util.MotorHashService;
import edu.ahs.robotics.util.loggers.DataLogger;

public class BaseTestAuto {
    private Ardennes ardennes;
    private Path path;
    private MecanumChassis chassis;
    private OBMCommand nullCommand = new NullCommand();
    private MotionConfig pathConfig;
    private DataLogger odometryLogger;

    public BaseTestAuto(List<Point> points, double initialPower, double finalPower, double[][] powers, MotionConfig pathConfig){
        MotorHashService.init();
        ardennes = new Ardennes();
        chassis = ardennes.getChassis();
        path = new Path(points, false, initialPower, finalPower, powers);//was 36
        chassis.setPosition(0,0, Math.PI); //FIXME
        odometryLogger = new DataLogger("odometryStats", "odometrySystem");

        chassis.startOdometrySystem();

        this.pathConfig = pathConfig;
    }


    public void afterStart(){
        chassis.followPath(path, pathConfig);
        chassis.stopMotors();
        FTCUtilities.sleep(2000);
        chassis.stopOdometrySystem();
    }
}
