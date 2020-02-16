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
     * <b>Currently Stripped down to only contain Position information.</b>
     * Ideal for containing any other info that could be determined by the OdometrySystem
     */
    class State {
       public Position position;
       //public Velocity velocity;
       //public double acceleration;


        public State(Position position) {
            this.position = new Position(position);

            //this.acceleration = acceleration;
            //this.velocity = new Velocity(velocity);
        }
    }
}
