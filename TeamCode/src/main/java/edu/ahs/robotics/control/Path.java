package edu.ahs.robotics.control;

import java.util.ArrayList;

public class Path {
    private final ArrayList<PointAtDistance> pointAtDistance;
    private double totalDistance = 0;

    public Path(ArrayList<Point> points) {
        pointAtDistance = new ArrayList<>();
        pointAtDistance.add(new PointAtDistance(points.get(0), 0, 0, 0));

        for (int i = 1; i < points.size(); i++) {
            Point current = points.get(i);
            Point previous = points.get(i - 1);
            double distanceFromPrevious = current.distanceTo(previous);
            totalDistance += distanceFromPrevious;
            pointAtDistance.add(new PointAtDistance(current, totalDistance, current.x - previous.x, current.y - previous.y));
        }
    }

    public PointAtDistance getPoint(int index) {
        return pointAtDistance.get(index);
    }

    /**
     * This method takes the bounding points and calculates a third point.
     * Then it creates a line, finds the intersection, and returns it as a point.
     * The third point is used to calculate distance from start and end because it allows for the robot to not be between the bounding points.
     *
     * @param robotPosition
     * @return Returns a location
     */
    public Location getTargetLocation(Position robotPosition) {
        //Find the 2 closest bounding points
        int boundingPoint = getBoundingPoints(robotPosition);
        PointAtDistance first = getPoint(boundingPoint);

        //Look to see if robot is past end of line (First bounding point is last point on path)
        if (boundingPoint == pointAtDistance.size() - 1) {
            Location loc = new Location(first);
            loc.pathFinished = true;
            return loc;
        }

        PointAtDistance second = getPoint(boundingPoint + 1);

        Location loc = new Location(second);

        //Find closest point on line to robot
        Line pathLine = new Line(first, second);
        loc.closestPoint = pathLine.getClosestPointOnLine(robotPosition);

        //Calculate distance to end and distance from start of path
        loc.distanceToEnd = (totalDistance - second.distance) + loc.closestPoint.distanceTo(second);
        loc.distanceFromStart = second.distance - loc.closestPoint.distanceTo(second);

        //Objective: Find distance to robot from path
        //Note: Positive distances are to the right of the path and negative are to the left
        //Step 1: Find perpendicular vector p to the heading
        double pX = loc.pathDeltaY;
        double pY = -loc.pathDeltaX;

        //Step 2: Calculate Robot vector
        double robotDeltaX = robotPosition.x - loc.closestPoint.x;
        double robotDeltaY = robotPosition.y - loc.closestPoint.y;

        //Step 3: Calculate dot product of p and robotVector normalised by length of path vector
        double pathVectorLength = Math.sqrt(Math.pow(loc.pathDeltaX, 2) + Math.pow(loc.pathDeltaY, 2));
        loc.distanceToRobot = ((pX * robotDeltaX) + (pY * robotDeltaY)) / pathVectorLength;

        return loc;
    }

    /**
     * This method finds the two closest points to the robot position and returns the points in order of path.
     *
     * @param robotPosition
     * @return indices of points. Use getPoint to find actual point.
     */
    public int getBoundingPoints(Position robotPosition) {
        int iFirst = 0;

        for (int i = 1; i < pointAtDistance.size(); i++) {
            PointAtDistance current = getPoint(i);
            double robotDeltaX = current.x - robotPosition.x;
            double robotDeltaY = current.y - robotPosition.y;

            double componentToCurrent = (robotDeltaX * current.pathDeltaX) + (robotDeltaY * current.pathDeltaY);
            if (componentToCurrent > 0) {
                break;
            } else {
                iFirst = i;
            }
        }

        return iFirst;
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
        private double pathDeltaX;
        private double pathDeltaY;

        public PointAtDistance(double x, double y, double distance, double pathDeltaX, double pathDeltaY) {
            super(x, y);
            this.distance = distance;
            this.pathDeltaX = pathDeltaX;
            this.pathDeltaY = pathDeltaY;

        }

        public PointAtDistance(Point p, double distance, double pathDeltaX, double pathDeltaY) {
            this(p.x, p.y, distance, pathDeltaX, pathDeltaY);
        }

    }

    public static class Location {
        public Point closestPoint;
        public double pathDeltaX;
        public double pathDeltaY;
        public double distanceToEnd;
        public double distanceFromStart;
        public double distanceToRobot;
        public boolean pathFinished;

        public Location(PointAtDistance pointAtDistance) {
            pathFinished = false;
            pathDeltaX = pointAtDistance.pathDeltaX;
            pathDeltaY = pointAtDistance.pathDeltaY;
        }
    }

}
