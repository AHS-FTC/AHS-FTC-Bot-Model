package edu.ahs.robotics.autopaths;

import edu.ahs.robotics.hardware.Executor;

public class OdometryPointTurn extends Motion{
    public double angle;
    public double maxPower;

    public OdometryPointTurn(Executor executor, double angle, double maxPower) {
        super(executor);
        this.angle = angle;
        this.maxPower = maxPower;
    }
}
