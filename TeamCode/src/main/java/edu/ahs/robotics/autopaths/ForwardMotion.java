package edu.ahs.robotics.autopaths;

import edu.ahs.robotics.hardware.Executor;

// Simple drive forward, can be interpreted by a chassis by itself or as a StraightMotion.
// For beginner code or debugging. Otherwise use StraightMotion
public class ForwardMotion extends StraightMotion {
    public double travelDistance;
    public double motorPower;
    public double timeOut;

    public ForwardMotion(double travelDistance, double motorPower, double timeOut, Executor executor) {
        super(travelDistance,motorPower,timeOut,executor,EncoderType.MOTOR_ENCODER, 0);
        this.travelDistance =  travelDistance;
        this.motorPower = motorPower;
        this.timeOut = timeOut;
    }

}
