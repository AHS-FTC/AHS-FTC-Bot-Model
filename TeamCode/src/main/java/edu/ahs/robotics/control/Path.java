package edu.ahs.robotics.control;

import java.util.ArrayList;

public class Path {
    private final ArrayList<PointAtDistance> pointAtDistance;
    double x;
    double y;
    private double maxVelocity;
    double totalTime = 0;
    double totalDistance = 0;

    public Path(ArrayList<Point> points) {
        pointAtDistance = new ArrayList<>();
        pointAtDistance.add(new PointAtDistance(points.get(0), 0));

        for (int i = 1; i < points.size(); i++) {

            Point current = points.get(i);
            Point previous = points.get(i-1);
            double distanceFromPrevious = current.distanceTo(previous);
            totalDistance += distanceFromPrevious;
            pointAtDistance.add(new PointAtDistance(current, totalDistance));
        }
    }

    public Position getTargetPosition(Position robotPosition) {
        PointAtDistance[] boundingPoints = getBoundingPoints(robotPosition);
        Point closest = boundingPoints[0];
        Point nextClosest = boundingPoints[1];
        Line pathLine = new Line(closest, nextClosest);

        return null;
        //return new Position(pathLine.findIntersection());
    }

    public PointAtDistance[] getBoundingPoints(Position robotPosition) {
        PointAtDistance closest = pointAtDistance.get(0);
        PointAtDistance nextClosest;
        double closestDistance = closest.distanceTo(robotPosition);
        double nextClosestDistance = Double.MAX_VALUE;

        for (int i = 1; i < pointAtDistance.size(); i++) {
            PointAtDistance currentPoint = this.pointAtDistance.get(i);
            double currentDistance = currentPoint.distanceTo(robotPosition);

            if (currentDistance < closestDistance) {
                nextClosestDistance = closestDistance;
                nextClosest = closest;
                closestDistance = currentDistance;
                closest = currentPoint;
            } else if (currentDistance < nextClosestDistance) {
                nextClosestDistance = currentDistance;
                nextClosest = currentPoint;
            } else {
                break;
            }
        }
        return null;
        //return new PointAtDistance[] {closest, nextClosest};
    }


//    public Position getTargetPosition(double currentTime) {
//        //math for target position based on totalDistance and interpolating
//        //is this where the d term goes?
//
//
//
//        if (currentTime > totalDistance) {
//            currentTime = totalDistance;
//        }
//
//        PointAtDistance previous = pointAtDistance.get(0);
//        PointAtDistance next = pointAtDistance.get(1);
//
//
//        for (int i = 1; i < pointAtDistance.size(); i++) {
//
//            next = pointAtDistance.get(i);
//            previous = pointAtDistance.get(i-1);
//
//            if (next.time >= currentTime) {
//                break;
//            }
//
//        }
//
//        //Interpolate between previous and current to find target position
//        double ratio = (currentTime - previous.time) / (next.time - previous.time);
//        double deltaX = next.getX() - previous.getX();
//        double deltaY = next.getY() - previous.getY();
//        double targetX = previous.getX() + (ratio * deltaX);
//        double targetY = previous.getY() + (ratio * deltaY);
//        double targetHeading = Math.atan(deltaY/deltaX);
//
//        return new Position(targetX,targetY,targetHeading);
//    }


    
    private static class PointAtDistance extends Point {
        private double distance;

        public PointAtDistance(double x, double y, double distance) {
            super(x, y);
            this.distance = distance;
            
        }

        public PointAtDistance(Point p, double distance) {
            this(p.x, p.y, distance);
        }

        public double getDistance() {
            return distance;
        }
    }


    

}
