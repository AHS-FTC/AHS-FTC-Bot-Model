package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.control.MotionConfig;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;

/**
 * Takes MotionConfig in use and changes the targetHeading at a specific location on the field.
 * Can be used at times where the heading of the robot should be changed mid movement.
 * @author Alex Appleby
 */
public class TargetHeadingChanger implements OBMCommand {
    private MotionConfig motionConfig;
    private double turnY;
    private double idealHeading;
    private boolean finished = false;

    public TargetHeadingChanger(MotionConfig motionConfig, double idealHeading, double turnY) {
        this.motionConfig = motionConfig;
        this.idealHeading = idealHeading;
        this.turnY = turnY;
    }

    @Override
    public boolean check(OdometrySystem.State robotState) {
        if(!finished){
            if(robotState.position.y > turnY){ //kind of static logic, can be changed if need be
                motionConfig.idealHeading = idealHeading;
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
