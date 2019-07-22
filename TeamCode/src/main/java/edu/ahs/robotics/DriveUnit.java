package edu.ahs.robotics;

public abstract class DriveUnit {
    private GearRatio gearRatio;
    private double wheelDiameter;
    private double wheelCircumference;
    String deviceName;
    MotorHashService.MotorTypes motorType;
    public abstract void setPower(double power);
    public abstract void zeroDistance();
    public abstract double getDistance();
}


