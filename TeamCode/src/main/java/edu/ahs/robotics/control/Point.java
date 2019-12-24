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
        return  Math.sqrt(Math.pow(x - p.x ,2) + Math.pow(y - p.y,2));
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}