package edu.ahs.robotics.autocommands.autopaths;

import edu.ahs.robotics.hardware.Executor;

public class ForwardMotionBasic extends ForwardMotion {//ForwardMotion but for lazy debugging
    public ForwardMotionBasic(double travelDistance, double motorPower, double timeOut, Executor executor) {
        super(travelDistance, motorPower, timeOut, executor);
    }
}
