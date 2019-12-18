package edu.ahs.robotics.control;

import edu.ahs.robotics.autocommands.autopaths.functions.Position;
import edu.ahs.robotics.hardware.DriveUnit;

/**
 * Functional class that manages positional PID in three global dimensions
 * @author Alex Appleby
 */
public class XYHeadingPID {
    private Config config;

    private double xErrorSum = 0, yErrorSum = 0, headingErrorSum = 0;

    /**
     * @param config An instance of the static Config class with configuration parameters
     */
    public XYHeadingPID(Config config) {
        this.config = config;
    }

    /**
     * Gets positional corrections using PID
     * @param current Global Position of the robot
     * @param target Global Target Position
     * @return Correction class containing global corrections for robot according to PID
     */
    public Correction getCorrection(Position current, Position target){
        double xCorrection = 0, yCorrection = 0, headingCorrection = 0;

        double xError = target.x - current.x;
        double yError = target.y - current.y;
        double headingError = target.heading - current.heading;

        xCorrection += xError * config.xP;
        yCorrection += yError * config.yP;
        headingCorrection += headingError * config.hP;

        xErrorSum += xError;
        yErrorSum += yError;
        headingErrorSum += headingError;

        xCorrection += xErrorSum * config.xI;
        yCorrection += yErrorSum * config.yI;
        headingCorrection += headingErrorSum * config.hI;

        return new Correction(xCorrection, yCorrection, headingCorrection);
    }

    public static class Correction{
        public double x, y, heading;

        public Correction(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    /**
     * Manages tuning parameters for XYH PID. Passed into XYHeadingPID constructor for init.
     */
    public static class Config {
         private double xP = 0, xI = 0, xD = 0;
         private double yP = 0, yI = 0, yD = 0;
         private double hP = 0, hI = 0, hD = 0;

         public void setXParameters(double p, double i, double d){
             xP = p;
             xI = i;
             xD = d;
         }

         public void setYParameters(double p, double i, double d){
            yP = p;
            yI = i;
            yD = d;
         }

         public void setHeadingParameters(double p, double i, double d) {
             hP = p;
             hI = i;
             hD = d;
         }
    }

}
