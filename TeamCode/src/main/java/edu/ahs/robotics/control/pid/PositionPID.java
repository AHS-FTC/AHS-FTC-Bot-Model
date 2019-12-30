package edu.ahs.robotics.control.pid;

import edu.ahs.robotics.control.Position;

/**
 * Functional class that manages positional PID in three global dimensions
 * @author Alex Appleby
 */
public class PositionPID {
    private Config config;

    private double xErrorSum = 0, yErrorSum = 0, headingErrorSum = 0;
    private double lastXError = 0, lastYError = 0, lastHeadingError = 0;

    /**
     * @param config An instance of the static Config class with configuration parameters
     */
    public PositionPID(Config config) {
        this.config = config;
    }

    /**
     * Gets positional corrections using PID
     * @param currentPosition Global Position of the robot
     * @param targetPosition Global Target Position
     * @return Correction class containing global corrections for robot according to PID
     */
    public Correction getCorrection(Position currentPosition, Position targetPosition){
        double xCorrection = 0, yCorrection = 0, headingCorrection = 0;

        LocalPosition localPosition = convertToLocalCoordinates(currentPosition, targetPosition); //recall, this position is relative to the axis of the robot

        double xError = localPosition.x; //previously targetPosition.x - currentPosition.x; local conversion is measured from origin.
        double yError = localPosition.y; //previously targetPosition.y - currentPosition.y;
        double headingError = targetPosition.heading - currentPosition.heading; // heading adjustments are still made in the global domain

        xCorrection += xError * config.xP;
        yCorrection += yError * config.yP;
        headingCorrection += headingError * config.hP;

        xErrorSum += xError;
        yErrorSum += yError;
        headingErrorSum += headingError;

        xCorrection += xErrorSum * config.xI;
        yCorrection += yErrorSum * config.yI;
        headingCorrection += headingErrorSum * config.hI;

        xCorrection += (lastXError - xError)*config.xD;
        yCorrection += (lastYError - yError)*config.yD;
        headingCorrection += (lastHeadingError - headingError)*config.hD;

        lastXError = xError;
        lastYError = yError;
        lastHeadingError = headingError;

        return new Correction(xCorrection, yCorrection, headingCorrection);
    }

    private LocalPosition convertToLocalCoordinates(Position robotPosition, Position targetPosition){
        double heading = robotPosition.heading;

        double globalDX = targetPosition.x - robotPosition.x;
        double globalDY = targetPosition.y - robotPosition.y;

        double x = Math.cos(heading) * globalDX + Math.sin(heading) * globalDY; //math here is derived from taking trig components
        double y = Math.cos(heading) * globalDY + Math.sin(heading) * globalDX; //x is forward

        return new LocalPosition(x,y);
    }

    /**
     * Position of target relative to the robot. Not to be confused with the global (and public) Position class.
     * Note that there isn't really any meaning to a 'local heading' so it's omitted
     */
    private class LocalPosition{

        private double x, y;

        private LocalPosition(double x, double y) {
            this.x = x;
            this.y = y;
        }
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
     * Manages tuning parameters for XYH PID. Passed into PositionPID constructor for init.
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
