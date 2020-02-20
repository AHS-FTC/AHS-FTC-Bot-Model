package edu.ahs.robotics.control.obm;

import edu.ahs.robotics.hardware.sensors.OdometrySystem;

/**
 * Interface for synchronous tasks to be ran by the robot as it drives.
 * Use this instead of a Thread, especially when you're 'writing' to hardware. Reads can get away with threads if necessary.
 * @author Alex Appleby
 */
public interface OBMCommand {

    /**
     * Method to synchronously run OBM tasks. Should not block.
     * @return boolean where true breaks higher running loop, ending motion.
     */
    boolean check(OdometrySystem.State robotState);
    void reset();
    boolean isFinished();
}
