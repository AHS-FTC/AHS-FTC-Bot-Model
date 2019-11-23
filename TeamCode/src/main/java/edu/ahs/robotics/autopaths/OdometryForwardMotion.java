package edu.ahs.robotics.autopaths;

import edu.ahs.robotics.hardware.Executor;

public class OdometryForwardMotion extends Motion {
    public double targetDistance; // in mm
    public double maxPower;

    public OdometryForwardMotion(Executor executor, double targetDistance, double maxPower) {
        super(executor);
        this.targetDistance = targetDistance;
        this.maxPower = maxPower;
    }
}
