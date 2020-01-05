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
       public double travelRadius;

        State(Position position, Velocity velocity, double travelRadius) {
            this.position = new Position(position);
            this.velocity = new Velocity(velocity);
            this.travelRadius = travelRadius;
        }
    }
}
