package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.hardware.sensors.OdometrySystem;

/**
 * OBMCommand for doing jack shit. When you need a filler OBMCommand for whatever reason.
 * @author Alex Appleby
 */
public class NullCommand implements OBMCommand {
    @Override
    public boolean check(OdometrySystem.State robotState) {
        //do nothing
        return false;
    }

    @Override
    public void reset() {
        //do nothing
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
