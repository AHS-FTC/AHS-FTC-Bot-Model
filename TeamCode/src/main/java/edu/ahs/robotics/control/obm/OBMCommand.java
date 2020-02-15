package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.hardware.sensors.OdometrySystem;

public interface OBMCommand {
    void check(OdometrySystem.State robotState);
    void reset();
    boolean isFinished();
}
