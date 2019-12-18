package edu.ahs.robotics.hardware;

import java.util.ArrayList;

import edu.ahs.robotics.autocommands.autopaths.functions.Position;

public class Path {
    private final ArrayList<PointAtTime> pointAtTimes;
    double x;
    double y;
    private double maxVelocity;
    double totalDistance = 0;
    double totalTime = 0;

    public Path(ArrayList<Point> points, double maxVelocity) {
        pointAtTimes = new ArrayList<>();
        pointAtTimes.add(new PointAtTime(points.get(0), 0));

        this.maxVelocity = maxVelocity;

        for (int i = 1; i < points.size(); i++) {
            Point current = points.get(i);
            Point previous = points.get(i-1);

            double distance = current.distanceTo(previous);
            totalDistance += distance;
            totalTime = calculateTimeAtDistance(totalDistance);

            pointAtTimes.add(new PointAtTime(current, totalTime));
        }
    }

    private double calculateTimeAtDistance(double distance) {
        return distance / maxVelocity;
    }

    /**
     * Finds the target location for the robot based on elapsed totalTime
     * @param currentTime
     * @return
     */
    public Position getTargetPosition(double currentTime) {
        //math for target position based on totalTime and interpolating
        //is this where the d term goes?

        if (currentTime > totalTime) {
            currentTime = totalTime;
        }

        PointAtTime previous = pointAtTimes.get(0);
        PointAtTime next = pointAtTimes.get(1);


        for (int i = 1; i < pointAtTimes.size(); i++) {

            next = pointAtTimes.get(i);
            previous = pointAtTimes.get(i-1);

            if (next.time >= currentTime) {
                break;
            }

        }

        //Interpolate between previous and current to find target position
        double ratio = (currentTime - previous.time) / (next.time - previous.time);
        double deltaX = next.getX() - previous.getX();
        double deltaY = next.getY() - previous.getY();
        double targetX = previous.getX() + (ratio * deltaX);
        double targetY = previous.getY() + (ratio * deltaY);
        double targetHeading = Math.atan(deltaY/deltaX);

        return new Position(targetX,targetY,targetHeading);
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
