package edu.ahs.robotics.hardware.sensors;

import edu.ahs.robotics.control.Position;
import edu.ahs.robotics.control.Velocity;

/**
 * Interface allowing OdometrySystem to be mocked without mocking out individual odometers.
 * @author Alex Appleby
 */
public interface OdometrySystem {
    Position getPosition();

    Velocity getVelocity();

    void start();

    void stop();

    boolean isRunning();

    void resetPosition(double x, double y, double heading);
}
