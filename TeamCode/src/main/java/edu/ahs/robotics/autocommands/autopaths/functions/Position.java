package edu.ahs.robotics.autocommands.autopaths.functions;

public class Position {
    public double x;
    public double y;
    public double heading;

    public Position(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public void setPosition(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
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
     * Measures the angle between the x axis and the line defined by the XY of both positions. Similar to atan2.
     * Angle follows standard conventions.
     * @param position
     * @return the angle in rads
     */
    public double angleTo(Position position){
        double dx = position.x - x;
        double dy = position.y - y;

        return Math.atan(dy/dx);
    }
}
