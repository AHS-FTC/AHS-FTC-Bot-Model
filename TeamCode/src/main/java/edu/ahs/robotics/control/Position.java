package edu.ahs.robotics.control;

/**
 * Represents the position of our robot. x and y coordinates are stored natively in inches
 * Heading is stored natively in radians
 */
public class Position {
    public double x;
    public double y;
    public double heading;

    public Position(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public void setHeading(double deltaX, double deltaY) {
        heading = Math.atan(deltaY/deltaX);
    }

    public double getHeadingInDegrees() {
        return Math.toDegrees(heading);
    }

    public void setPosition(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
