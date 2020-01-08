package edu.ahs.robotics.control;

/**
 * Represents the position of our robot. x and y coordinates are stored natively in inches.
 * Contains an internal Point.
 * Heading is stored natively in radians
 * @author Alex and Andrew
 */
public class Position {
    public double x;
    public double y;

    /**
     * Heading of a robot position stored in radians. 0 rad faces the global x axis, where bot local and global coordinates align.
     * Does not wrap, can exceed pi, 2pi, -pi, -2pi radians.
     */
    public double heading;

    public Position(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /**
     * Copy Constructor
     */
    public Position(Position p){
        copyFrom(p);
    }

    /**
     * Creates a position given a cartesian point and a heading in radians.
     * @param point
     * @param heading
     */
    public Position(Point point, double heading){
        this(point.x, point.y, heading);
    }

    /**
     * Sets position fields based on another position
     */
    public void copyFrom(Position p){
        this.x = p.x;
        this.y = p.y;
        this.heading = p.heading;
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
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Point getAsPoint(){
        return new Point(x,y);
    }

    /**
     * Measures the standard two-dimensional heading between this instance of position and another. Omits heading.
     * @param targetPosition the other position
     * @return the scalar distance to the other position
     */
    public double distanceTo(Position targetPosition){
        double xDistance = targetPosition.x - x;
        double yDistance = targetPosition.y - y;

        return Math.sqrt(xDistance * xDistance  +  yDistance * yDistance); //distance formula
    }


    /**
     * Borrows angleTo() method in Point and omits heading.
     * @return angle in radians.
     */
    public double angleTo(Position position){ return getAsPoint().angleTo(position.getAsPoint());
    }
}
