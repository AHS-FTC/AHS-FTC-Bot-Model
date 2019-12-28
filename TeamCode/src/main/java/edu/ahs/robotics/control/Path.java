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

    public PointAtDistance getPoint(int index) {
        return pointAtDistance.get(index);
    }

    /**
     * This method takes the bounding points and calculates a third point.
     * Then it creates a line and gets the intersection and returns it as a point.
     * The third point is used to calculate distance from start and end because it allows for the robot to not be between the bounding points.
     * @param robotPosition
     * @return Returns a location that is used in Line
     */
    public Location getTargetLocation(Position robotPosition) {
        int[] boundingPoints = getBoundingPoints(robotPosition);
        PointAtDistance first = getPoint(boundingPoints[0]);
        PointAtDistance second = getPoint(boundingPoints[1]);

        //Calculate third point and clip if it is off end of path
        int nextIndex = boundingPoints[1] + 1;
        if (nextIndex > pointAtDistance.size()-1){
            nextIndex = pointAtDistance.size()-1;
        }
        PointAtDistance third = getPoint(nextIndex);

        Line pathLine = new Line(first, second);
        Point closestPointOnLine = pathLine.getClosestPointOnLine(robotPosition);

        double deltaX = second.x - first.x;
        double deltaY = second.y - first.y;

        //Use third point to calculate
        double distanceToEnd = totalDistance - third.distance + closestPointOnLine.distanceTo(third);
        double distanceFromStart = third.distance - closestPointOnLine.distanceTo(third);

        return new Location(closestPointOnLine, deltaX, deltaY, distanceToEnd, distanceFromStart);
    }

    /**
     * This method finds the two closest points to the robot position and returns the points in order of path.
     * @param robotPosition
     * @return indices of points. Use getAsPoint to find actual point.
     */
    public int[] getBoundingPoints(Position robotPosition) {
        int closest = 0;
        int nextClosest = 1;
        double closestDistance = pointAtDistance.get(closest).distanceTo(robotPosition);
        double nextClosestDistance = Double.MAX_VALUE;

        for (int i = 1; i < pointAtDistance.size(); i++) {
            PointAtDistance currentPoint = this.pointAtDistance.get(i);
            double currentDistance = currentPoint.distanceTo(robotPosition);

            if (currentDistance < closestDistance) {
                nextClosestDistance = closestDistance;
                nextClosest = closest;
                closestDistance = currentDistance;
                closest = i;
            } else if (currentDistance < nextClosestDistance) {
                nextClosestDistance = currentDistance;
                nextClosest = i;
            } else {
                break;
            }
        }
        //Return the points in order of path
        if (closest < nextClosest) {
            return new int[]{closest, nextClosest};
        } else {
            return new int[]{nextClosest, closest};
        }
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


    
    public static class PointAtDistance extends Point {
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

    public static class Location {
        public double deltaX;
        public double deltaY;
        public double distanceToEnd;
        public double distanceFromStart;
        public Point point;

        public Location(Point point, double deltaX, double deltaY, double distanceToEnd, double distanceFromStart) {
            this.point = point;
            this.deltaX = deltaX;
            this.deltaY = deltaY;
            this.distanceToEnd = distanceToEnd;
            this.distanceFromStart = distanceFromStart;
        }

    }

}
