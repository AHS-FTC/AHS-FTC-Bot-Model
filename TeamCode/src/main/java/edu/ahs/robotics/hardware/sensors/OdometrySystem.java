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

    Odometer getBackOdometer();

    /**
     * Contains atomic and threadsafe information on the current state of the odometry system.
     */
    class State {
       public Position position;
       public Velocity velocity;
       public double acceleration;
        /**
         * Measures the signed radius of travel in the x dimension.
         * Sign contains information on the directional nature of the travel. A positive radius indicates a leftward arc, while a negative radius indicates a rightward arc.
         * For non-tank style movement the idea of travel radius will need to be elaborated on.
         */
       public double travelRadius;

        public State(Position position, Velocity velocity, double acceleration, double travelRadius) {
            this.position = new Position(position);
            this.velocity = new Velocity(velocity);
            this.travelRadius = travelRadius;
            this.acceleration = acceleration;
        }
    }
}
