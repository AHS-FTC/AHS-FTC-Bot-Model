package edu.ahs.robotics.control;

/**
 * Class that contains velocity information in vector form
 * @author Alex Appleby
 */
public class Velocity {
    public double speed;
    public double direction;


    /**
     * Only accessible in-class to prevent signature conflict
     */
    private Velocity(double speed, double direction) {
        this.speed = speed;
        this.direction = direction;
    }

    /**
     * Acts as a constructor to resolve signature conflicts
     * @param speed A.K.A magnitude in inches/second
     * @param direction in radians, follows established conventions
     */
    public static Velocity makeVelocity(double speed, double direction){
        return new Velocity(speed, direction);
    }

    /**
     * Creates vector using components. Acts as a constructor to resolve signature conflicts.
     * @param dx the global x component of the velocity vector
     * @param dy the global y component of the velocity vector
     */
    public static Velocity makeVelocityFromDxDy(double dx, double dy){
        double speed = Math.sqrt((dx * dx) + (dy * dy)); //distance formula
        double direction = Math.atan2(dy , dx); // atan2 good

        if (direction < 0){ // negative bad
            direction += 2 * Math.PI;
        }

        return new Velocity(speed, direction);
    }

    public void setVelocity(double speed, double direction){
        this.speed = speed;
        this.direction = direction;
    }

}
