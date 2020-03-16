package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.control.MotionConfig;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;

/**
 * Takes MotionConfig in use and changes the targetHeading at a specific location on the field.
 * Can be used at times where the heading of the robot should be changed mid movement.
 * @author Alex Appleby
 */
public class GlobalHeadingChanger implements OBMCommand {
    private MotionConfig motionConfig;
    private double turnY;
    private double globalHeading;
    private boolean finished = false;

    public GlobalHeadingChanger(MotionConfig motionConfig, double globalHeading, double turnY) {
        this.motionConfig = motionConfig;
        this.globalHeading = globalHeading;
        this.turnY = turnY;
    }

    @Override
    public boolean check(OdometrySystem.State robotState) {
        if(!finished){
            if(robotState.position.y > turnY){ //kind of static logic, can be changed if need be
                motionConfig.usingGlobalHeading = true;
                motionConfig.globalHeading = globalHeading;
                finished = true;
            }
        }
        return false;
    }

    @Override
    public void reset() {
        finished = false;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
