package edu.ahs.robotics.autocommands.autopaths;

import edu.ahs.robotics.hardware.Executor;

public class OdometryMotion extends Motion{
    public double leftTarget;
    public double rightTarget;
    public double maxPower;

    public OdometryMotion(Executor executor, double maxPower, double leftTarget, double rightTarget) {
        super(executor);
        this.leftTarget = leftTarget;
        this.rightTarget = rightTarget;
        this.maxPower = maxPower;
    }
}
