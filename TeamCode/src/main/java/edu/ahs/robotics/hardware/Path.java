package edu.ahs.robotics.hardware;

import java.util.ArrayList;
import java.util.Iterator;

import edu.ahs.robotics.autocommands.autopaths.functions.Position;

public class Path {
    private final ArrayList<PointAtTime> pointAtTimes;
    double x;
    double y;
    private static final double MAX_VELOCITY = 1;

    public Path(ArrayList<Point> points) {
        pointAtTimes = new ArrayList<>();
        pointAtTimes.add(new PointAtTime(points.get(0), 0));
        double totalDistance = 0;

        for (int i = 1; i < points.size(); i++) {
            Point current = points.get(i);
            Point previous = points.get(i-1);

            double distance = current.distanceTo(previous);
            totalDistance += distance;
            double time = totalDistance / MAX_VELOCITY;

            pointAtTimes.add(new PointAtTime(current, time));
        }
    }

    /**
     * Finds the target location for the robot based on elapsed time
     * @param currentTime
     * @return
     */
    public Position getTargetPosition(double currentTime) {
        //math for target position based on time and interpolating
        //is this where the d term goes?
        PointAtTime last = pointAtTimes.get(0);

        for (int i = 1; i < pointAtTimes.size(); i++) {

            PointAtTime next = pointAtTimes.get(i);
            if (next.time >= currentTime) {
                //Interpolate between last and current to find target position
                double ratio = (currentTime - last.time) / (next.time - last.time);
                double deltaX = next.getX() - last.getX();
                double deltaY = next.getY() - last.getY();
                double targetX = last.getX() + (ratio * deltaX);
                double targetY = last.getY() + (ratio * deltaY);
                double targetHeading = Math.atan(deltaY/deltaX);
                return new Position(targetX,targetY,targetHeading);
            }
        }
        return null;
    }

    public static class Point {
        private double x;
        private double y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double distanceTo(Point p) {
            return Math.sqrt(Math.pow(x - p.x ,2) + Math.pow(y - p.y,2));
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }
    }
    
    private static class PointAtTime extends Point {
        private double time;

        public PointAtTime(double x, double y, double time) {
            super(x, y);
            this.time = time;
            
        }

        public PointAtTime(Point p, double time) {
            this(p.x, p.y, time);
        }

        public double getTime() {
            return time;
        }
    }
    

}
