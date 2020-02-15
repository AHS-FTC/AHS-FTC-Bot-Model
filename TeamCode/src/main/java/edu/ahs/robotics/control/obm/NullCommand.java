package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.hardware.sensors.OdometrySystem;

public class NullCommand implements OBMCommand {
    @Override
    public void check(OdometrySystem.State robotState) {
        //do nothing
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
