package edu.ahs.robotics.hardware.sensors;

import com.qualcomm.robotcore.hardware.DcMotor;

import edu.ahs.robotics.util.FTCUtilities;

public class Odometer {
    private DcMotor motor;
    private double wheelCircumference;// in mm
    private final double TICKS_PER_ROTATION = 1440;
    private int direction = 1;

    public Odometer(String deviceName, double wheelDiameter, boolean flip) {
        motor = FTCUtilities.getMotor(deviceName);
        reset();
        wheelCircumference = wheelDiameter*Math.PI;
        if(flip){
            direction = -1;
        }
    }

    public double getDistance(){
        int ticks = motor.getCurrentPosition();
        double distance = (ticks*wheelCircumference)/TICKS_PER_ROTATION;
        return direction*distance;
    }

    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getRotations(){
        int ticks = motor.getCurrentPosition();
        double rotations = ticks/TICKS_PER_ROTATION;
        return direction*rotations;
    }
}
