package edu.ahs.robotics.control;

import java.util.Objects;

public class Point {
    public double x;
    public double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Point point = (Point) o;
        return Double.compare(point.x, x) == 0 &&
                Double.compare(point.y, y) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }

    public double distanceTo(Point p) {
        return Math.sqrt(Math.pow(x - p.x ,2) + Math.pow(y - p.y,2));
    }

    public double distanceTo(Position p) {
        return  distanceTo(p.getAsPoint());
    }

    /**
     * Measures the angle between the x axis and the line defined by the XY of both positions.
     * Angle follows standard conventions.
     * @return the angle in rads
     */
    public double angleTo(Point p){ //note that this method is primarily tested in the Position testing class, where the tests lived before being transferred over
        double dx = p.x - x;
        double dy = p.y - y;

        double angle = Math.atan2(dy,dx);

        if(angle < 0){
            return angle + (2 * Math.PI); //only return positive angles
        } else {
            return Math.atan2(dy, dx);
        }
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}