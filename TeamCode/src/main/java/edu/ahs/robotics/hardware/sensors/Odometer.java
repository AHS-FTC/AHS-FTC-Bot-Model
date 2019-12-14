package edu.ahs.robotics.hardware.sensors;

/**
 * Interface for odometers to enable mocking of the sensor
 * @author Alex Appleby
 */
public interface Odometer {
    double getDistance();

    void reset();
}
