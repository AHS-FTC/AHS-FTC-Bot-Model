package edu.ahs.robotics.control;

/**
 * Class that contains velocity information in vector form
 * @author Alex Appleby
 */
public class Velocity {
    public double dx;
    public double dy;

    //public double speed;
    //public double direction;


    /**
     * Only accessible in-class to prevent signature conflict
     */
    private Velocity(double dx, double dy) {
        this.dx = dx;
        this.dy = dy;
    }

    /**
     * Acts as a constructor to resolve signature conflicts
     * @param speed A.K.A magnitude in inches/second
     * @param direction in radians, follows established conventions
     */
    public static Velocity makeVelocityFromSpeedDirection(double speed, double direction){
        double dx = speed * Math.cos(direction);
        double dy = speed * Math.sin(direction);

        return new Velocity(dx, dy);
    }

    /**
     * Creates vector using components. Acts as a constructor to resolve signature conflicts.
     * @param dx the global x component of the velocity vector
     * @param dy the global y component of the velocity vector
     */
    public static Velocity makeVelocity(double dx, double dy){
        return new Velocity(dx, dy);
    }

    public void setVelocity(double speed, double direction){
        double dx = speed * Math.cos(direction);
        double dy = speed * Math.sin(direction);

        this.dx = dx;
        this.dy = dy;
    }

    public double speed(){
        return getMagnitude(); //distance formula
    }

    public double direction(){
        double direction = Math.atan2(dy , dx); // atan2 good

        if(direction < 0){
            direction += (2 * Math.PI);
        }

        return direction;
    }

    private double getMagnitude(){
        return Math.sqrt((dx * dx) + (dy * dy));
    }

    public void scaleMagnitude(double magnitude){
        double magnitudeRatio = magnitude/this.getMagnitude();

        dx = dx * magnitudeRatio; //by similar triangles
        dy = dy * magnitudeRatio;
    }

}
