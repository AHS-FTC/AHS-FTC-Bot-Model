package edu.ahs.robotics.autopaths;

import edu.ahs.robotics.hardware.Executor;

public class PointTurn extends Motion {
    public double targetHeading;//delta for relative, absolute for absolute
    public Type type;
    public double power;

    public enum Type{
        RELATIVE, //turns x degrees from current targetHeading
        ABSOLUTE; //turns to targetHeading x degrees
    }

    public PointTurn(Executor executor, double targetHeading, double power, Type type) {
        super(executor);
        this.targetHeading = targetHeading;
        this.type = type;
        if(Math.abs(power)<=1){//maxPower should be between 1 and -1, duh
            this.power = power;
        } else {
            try {
                throw new Exception("PointTurn Power is out of bounds");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public PointTurn(Executor executor, double targetHeading, double power) {
        this(executor, targetHeading, power, Type.RELATIVE);
    }
}
