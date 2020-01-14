package edu.ahs.robotics.control;

public class Vector {
    public double x, y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Makes a unit vector for a given heading.
     * @param direction Standard cartesian angle in radians.
     * @return A vector of length 1 in the specified direction.
     */
    public static Vector makeUnitVector(double direction){
        double x = Math.cos(direction);
        double y = Math.sin(direction);
        return new Vector(x,y);
    }

    public Vector getPerpVector(){
        return new Vector(-y,x);
    }

    /**
     * Inverts this vector, effectively multiplying by -1, retaining magnitude but flipping direction.
     * @return This instance, for chaining.
     */
    public Vector invert(){
        y = -y;
        x = -x;
        return this;
    }
    /**
     * Scales this vector by a constant, retaining direction but scaling magnitude.
     * @param scale The scalar constant. Can be negative.
     * @return This instance, for chaining.
     */
    public Vector scale(double scale){
        y = y * scale;
        x = x * scale;
        return this;
    }
}
