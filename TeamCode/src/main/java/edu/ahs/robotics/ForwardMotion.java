package edu.ahs.robotics;

public class ForwardMotion extends Motion { // Simple drive forward
    public final double travelDistance;
    public final double motorPower;
    public final double timeOut;

    public ForwardMotion(double travelDistance, double motorPower, double timeOut, Executor executor) {
        super(executor);
        this.travelDistance =  travelDistance;
        this.motorPower = motorPower;
        this.timeOut = timeOut;
    }

}
