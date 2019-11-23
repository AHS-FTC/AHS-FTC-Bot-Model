package edu.ahs.robotics.autocommands.autopaths;

import edu.ahs.robotics.hardware.Executor;

public class StraightMotion extends Motion{
    public double travelDistance;
    public double motorPower;
    public double heading;//0 is forward
    public double timeOut;
    public EncoderType encoderType;

    public enum EncoderType{
        ODOMETRY_DIRECT,//Measures Odometry wheel directly. Assumes that robot moves in direction of Wheel.
        ODOMETRY_LOCALIZED,//Uses Odometry Localization to track position.
        MOTOR_ENCODER,//Uses motor encoders
    }
    public StraightMotion(double travelDistance, double motorPower, double timeOut, Executor executor, EncoderType encoderType, double heading) {
        super(executor);
        this.travelDistance =  travelDistance;
        this.motorPower = motorPower;
        this.timeOut = timeOut;
        this.encoderType = encoderType;
    }
}
