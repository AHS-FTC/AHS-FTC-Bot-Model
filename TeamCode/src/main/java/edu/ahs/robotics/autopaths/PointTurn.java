package edu.ahs.robotics.autopaths;

import edu.ahs.robotics.hardware.Executor;

public class PointTurn extends Motion {
    public double heading;//delta for relative, absolute for absolute
    public Type type;

    public enum Type{
        RELATIVE, //turns x degrees from current heading
        ABSOLUTE; //turns to heading x degrees
    }

    public PointTurn(Executor executor, double heading, double power, Type type) {
        super(executor);
        this.heading = heading;
        this.type = type;
    }

    public PointTurn(Executor executor, double heading, double power) {
        this(executor, heading, power, Type.RELATIVE);
    }
}
