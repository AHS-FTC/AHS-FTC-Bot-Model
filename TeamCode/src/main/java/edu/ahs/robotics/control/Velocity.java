package edu.ahs.robotics.control;

/**
 * Class that contains velocity information in vector form
 * @author Alex Appleby
 */
public class Velocity {
    public double speed;
    public double direction;

    /**
     * @param speed A.K.A magnitude in inches/second
     * @param direction in radians, follows established conventions
     */
    public Velocity(double speed, double direction) {
        this.speed = speed;
        this.direction = direction;
    }

    public void setVelocity(double speed, double direction){
        this.speed = speed;
        this.direction = direction;
    }
}
