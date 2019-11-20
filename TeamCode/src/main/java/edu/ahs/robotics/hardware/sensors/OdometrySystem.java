package edu.ahs.robotics.hardware.sensors;

import com.qualcomm.robotcore.hardware.DcMotor;

import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.autopaths.functions.Position;

public class OdometrySystem {
    private IMU imu;
    private DcMotor xMotor, yMotor;
    private boolean isRunning = false;

    private int xTicks = 0, yTicks = 0;
    private Position position;

    private final double TICKS_PER_ROTATION = 360;
    private double wheelCircumference;

    public OdometrySystem(DcMotor x, DcMotor y, IMU imu, double wheelCircumference) {
        xMotor = x;
        yMotor = y;
        this.imu = imu;
        this.wheelCircumference = wheelCircumference;

        //takes starting coordinates of field
        position = new Position(0,0,0); //Todo set this for starting position of field
    }

    public void start (){
        isRunning = true;
        double xRotations, yRotations;
        double xDistance, yDistance;

        while(isRunning){

            xTicks = xMotor.getCurrentPosition();
            yTicks = yMotor.getCurrentPosition();

            FTCUtilities.OpLogger("xTicks", xTicks);
            FTCUtilities.OpLogger("yTicks", yTicks);

            xRotations = xTicks/TICKS_PER_ROTATION;
            yRotations = yTicks/TICKS_PER_ROTATION;


            //xDistance = xRotations;


        }
    }

    public void stop(){
        isRunning = false;
    }

    public Position getPosition(){
        return position;
    }

}
