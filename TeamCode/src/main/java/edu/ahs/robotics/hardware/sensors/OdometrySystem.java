package edu.ahs.robotics.hardware.sensors;

import edu.ahs.robotics.control.Position;
import edu.ahs.robotics.control.Velocity;

/**
 * Interface allowing OdometrySystem to be mocked without mocking out individual odometers.
 * @author Alex Appleby
 */
public interface OdometrySystem {
    State getState();

    void start();

    void stop();

    boolean isRunning();

    void setPosition(double x, double y, double heading);

    Odometer getX1Odometer();

    Odometer getX2Odometer();

    /**
     * Contains atomic and threadsafe information on the current state of the odometry system.
     */
    class State {
       public Position position;
       public Velocity velocity;
        /**
         * Measures the current curvature of travel in the x dimension.
         * Calculated as 1/r, where traveling with a small radius means high curvature and infinite radius yields zero curvature.
         * For non-tank style movement the idea of curvature will need to be elaborated on.
         */
       public double travelCurvature;

        State(Position position, Velocity velocity, double travelCurvature) {
            this.position = new Position(position);
            this.velocity = new Velocity(velocity);
            this.travelCurvature = travelCurvature;
        }
    }
}
