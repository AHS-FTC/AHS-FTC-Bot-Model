package edu.ahs.robotics.control;

import java.util.ArrayList;

public class Path {
    static final double LOOK_AHEAD_DISTANCE = 6.0; /*Package visible for testing*/
    private final ArrayList<PointAtDistance> pointAtDistance;
    private double totalDistance = 0;


    /* Package visible for testing*/ int iFirstBoundingPoint = 0;
    private double minRampUpSpeed;
    private double minRampDownSpeed;
    private double maxVelocity;

    public Path(ArrayList<Point> points, double minRampUpSpeed, double minRampDownSpeed, double maxVelocity) {
        pointAtDistance = new ArrayList<>();
        pointAtDistance.add(new PointAtDistance(points.get(0), 0, 0, 0, 0));

        this.minRampUpSpeed = minRampUpSpeed;
        this.minRampDownSpeed = minRampDownSpeed;
        this.maxVelocity = maxVelocity;

        for (int i = 1; i < points.size(); i++) {
            Point current = points.get(i);
            Point previous = points.get(i - 1);

            if (current.x == previous.x && current.y == previous.y) {
                continue;
            }

            double distanceFromPrevious = current.distanceTo(previous);
            totalDistance += distanceFromPrevious;
            pointAtDistance.add(new PointAtDistance(current, totalDistance, current.x - previous.x, current.y - previous.y, distanceFromPrevious));
        }
    }

    public PointAtDistance getPoint(int index) {
        return pointAtDistance.get(index);
    }

    public boolean isFinished(Position robotPosition) {
        double componentAlongPath = getComponentAlongPath(robotPosition, pointAtDistance.get(pointAtDistance.size() - 1));
        return componentAlongPath <= 0;
    }

    /**
     *
     * @param robotPosition
     * @return Returns a location
     */
    public Location getTargetLocation(Position robotPosition) {
        //Find the 2 closest bounding points
        updateFirstBoundingPoint(robotPosition);
        PointAtDistance first = getPoint(iFirstBoundingPoint);

        PointAtDistance second = getPoint(iFirstBoundingPoint + 1);

        Location loc = new Location(second);

        //Find closest point on line to robot
        Line pathLine = new Line(first, second);
        loc.closestPoint = pathLine.getClosestPointOnLine(robotPosition);

        //Calculate distance to end and distance from start of path
        loc.distanceToEnd = (totalDistance - second.distanceFromStart) + loc.closestPoint.distanceTo(second);
        loc.distanceFromStart = second.distanceFromStart - loc.closestPoint.distanceTo(second);

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

        //Calculate speed at location
        loc.speed = getTargetSpeed(loc.distanceFromStart);

        return loc;
    }

    private double getTargetSpeed(double distanceFromStart) {
        double upScale = 2; //Todo adjust
        double downScale = 1; //Todo adjust
        if (distanceFromStart > totalDistance){
            distanceFromStart = totalDistance;
        }
        double distanceToEnd = totalDistance - distanceFromStart;
        double rampDown = (downScale * distanceToEnd) + minRampDownSpeed;
        double rampUp = (upScale * distanceFromStart) + minRampUpSpeed;
        return Math.min(rampDown, Math.min(rampUp, maxVelocity));
    }

    /**
     * Finds the closest point behind the robot in direction of travel and updates the first point
     * @param robotPosition
     */
    public void updateFirstBoundingPoint(Position robotPosition) {

        for (int i = iFirstBoundingPoint + 1; i < pointAtDistance.size(); i++) {
            PointAtDistance current = getPoint(i);
            double componentToCurrent = getComponentAlongPath(robotPosition, current);
            if (componentToCurrent > 0) {
                break;
            } else {
                iFirstBoundingPoint = i;
            }
        }
    }

    /**
     * Takes a position and a path point and finds the component of the distance along the path between the position and the path point
     * @param robotPosition
     * @param pathPoint
     * @return Non-normalized dotProduct
     */
    private double getComponentAlongPath(Position robotPosition, PointAtDistance pathPoint) {
        double robotDeltaX = pathPoint.x - robotPosition.x;
        double robotDeltaY = pathPoint.y - robotPosition.y;

        return (robotDeltaX * pathPoint.pathDeltaX) + (robotDeltaY * pathPoint.pathDeltaY);
    }

    public static class PointAtDistance extends Point {
        private double distanceFromStart;
        private double pathDeltaX;
        private double pathDeltaY;
        private double distanceToPrevious;

        public PointAtDistance(double x, double y, double distanceFromStart, double pathDeltaX, double pathDeltaY, double distanceToPrevious) {
            super(x, y);
            this.distanceFromStart = distanceFromStart;
            this.pathDeltaX = pathDeltaX;
            this.pathDeltaY = pathDeltaY;
            this.distanceToPrevious= distanceToPrevious;

        }

        public PointAtDistance(Point p, double distanceFromStart, double pathDeltaX, double pathDeltaY, double distanceToPrevious) {
            this(p.x, p.y, distanceFromStart, pathDeltaX, pathDeltaY, distanceToPrevious);
        }

    }

    public static class Location {
        public Point closestPoint;
        public double pathDeltaX;
        public double pathDeltaY;
        public double distanceToEnd;
        public double distanceFromStart;
        public double distanceToRobot;
        public double pathSegmentLength;
        public double speed;

        public Location(PointAtDistance pointAtDistance) {
            pathDeltaX = pointAtDistance.pathDeltaX;
            pathDeltaY = pointAtDistance.pathDeltaY;
            pathSegmentLength = pointAtDistance.distanceToPrevious;
        }
    }

}
