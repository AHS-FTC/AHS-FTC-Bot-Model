package edu.ahs.robotics.control;

/**
 * Represents the position of our robot. x and y coordinates are stored natively in inches.
 * Contains an internal Point.
 * Heading is stored natively in radians
 * @author Alex and Andrew
 */
public class Position {
    private Point point;

    /**
     * Heading of a robot position stored in radians. 0 rad faces the global x axis, where bot local and global coordinates align.
     * Does not wrap, can exceed pi, 2pi, -pi, -2pi radians.
     */
    public double heading;

    public Position(double x, double y, double heading) {
        this(new Point(x,y), heading); // call the other 'officialer' constructor
    }

    /**
     * Creates a position given a cartesian point and a heading in radians.
     * @param point
     * @param heading
     */
    public Position(Point point, double heading){
        this.point = point;
        this.heading = heading;
    }

    /**
     * Sets the position heading given a dx and dy using the Math.atan2() method.
     */
    public void setHeading(double deltaX, double deltaY) {
        heading = Math.atan(deltaY/deltaX);
    }

    /**
     * Converts native radian heading measurement into degrees.
     */
    public double getHeadingInDegrees() {
        return Math.toDegrees(heading);
    }

    /**
     * Overrides all position fields. Obeys standard axes conventions.
     * @param x Cartesian x pos
     * @param y Cartesian y pos
     * @param heading Direction of robot in radians
     */
    public void setPosition(double x, double y, double heading){
        point.x = x;
        point.y = y;
        this.heading = heading;
    }

    public double x(){
        return point.x;
    }

    public double y(){
        return point.y;
    }

    /**
     * Increments the position x value by a double amount. Utilized primarily in OdometrySystemImpl.
     * @param increment How much is being added to x. Can be negative.
     */
    public void incrementX(double increment){
        point.x += increment;
    }

    /**
     * Increments the position y value by a double amount. Utilized primarily in OdometrySystemImpl.
     * @param increment How much is being added to y. Can be negative.
     */
    public void incrementY(double increment){
        point.y += increment;
    }

    public Point getAsPoint(){
        return point;
    }

    /**
     * Measures the standard two-dimensional heading between this instance of position and another. Omits heading.
     * @param targetPosition the other position
     * @return the scalar distance to the other position
     */
    public double distanceTo(Position targetPosition){
        double xDistance = targetPosition.x() - point.x;
        double yDistance = targetPosition.y() - point.x;

        return Math.sqrt(xDistance * xDistance  +  yDistance * yDistance); //distance formula
    }


    /**
     * Borrows angleTo() method in point and omits heading.
     * @return angle in radians.
     */
    public double angleTo(Position position){
        return point.angleTo(position.getAsPoint());
    }
}
