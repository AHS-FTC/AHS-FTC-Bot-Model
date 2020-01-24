package edu.ahs.robotics.util;

/**
 * Contains Useful Math Methods for general use
 */
public class FTCMath {
    /**
     * Wraps an angle to the range -2pi to 2pi
     * @param angle Angle in radians
     */
    public static double wrapAngle(double angle){
        return angle % (2*Math.PI);
    }

    /**
     * Ensures that an angle is expressed in the shortest distance from 0 radians as possible. Generally useful for angle differences.
     * @param angle Angle in radians.
     */
    public static double ensureIdealAngle(double angle){
        while (angle > Math.PI){
            angle -= (2*Math.PI);
        }
        while (angle < -Math.PI){
            angle += (2*Math.PI);
        }
        return angle;
    }
}
