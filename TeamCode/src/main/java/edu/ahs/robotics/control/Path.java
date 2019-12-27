package edu.ahs.robotics.control;

import java.util.ArrayList;

public class Path {
    private final ArrayList<PointAtDistance> pointAtDistance;
    private double totalDistance = 0;

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
     * Then it creates a line, finds the intersection, and returns it as a point.
     * The third point is used to calculate distance from start and end because it allows for the robot to not be between the bounding points.
     * @param robotPosition
     * @return Returns a location
     */
    public Location getTargetLocation(Position robotPosition) {
        Location loc = new Location();

        //Find the 2 closest bounding points
        int[] boundingPoints = getBoundingPoints(robotPosition);
        PointAtDistance first = getPoint(boundingPoints[0]);
        PointAtDistance second = getPoint(boundingPoints[1]);

        //Calculate Path vector using bounding points
        loc.pathDeltaX = second.x - first.x;
        loc.pathDeltaY = second.y - first.y;

        //Find closest point on line to robot
        Line pathLine = new Line(first, second);
        loc.closestPoint = pathLine.getClosestPointOnLine(robotPosition);

        //Objective: Calculate distance to end and distance from start of path
        //Step 1: Find next (third) point on path in case robot is outside bounding points eg. Off start or end of path
        int nextIndex = boundingPoints[1] + 1;
        if (nextIndex > pointAtDistance.size()-1){
            nextIndex = pointAtDistance.size()-1;
        }
        PointAtDistance third = getPoint(nextIndex);

        //Step 2: Use third point to calculate distances
        loc.distanceToEnd = totalDistance - third.distance + loc.closestPoint.distanceTo(third);
        loc.distanceFromStart = third.distance - loc.closestPoint.distanceTo(third);

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
        loc.distanceToRobot = ((pX * robotDeltaX) + (pY * robotDeltaY))/ pathVectorLength;

        //Objective: Look to see if the robot is at or past last point
        //Note: If last point is second point of our bounding range then return that the path is finished
        loc.pathFinished = false;
        PointAtDistance lastPoint = getPoint(pointAtDistance.size() -1);
        if (lastPoint == second) {

            double robotVectorToLastX = lastPoint.x - robotPosition.x;
            double robotVectorToLastY = lastPoint.y - robotPosition.y;

            if ((robotVectorToLastX * loc.pathDeltaX) + (robotVectorToLastY * loc.pathDeltaY) <= 0) {
                loc.pathFinished = true;
            }
        }

        return loc;
    }

    /**
     * This method finds the two closest points to the robot position and returns the points in order of path.
     * @param robotPosition
     * @return indices of points. Use getPoint to find actual point.
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

    }

    public static class Location {
        public Point closestPoint;
        public double pathDeltaX;
        public double pathDeltaY;
        public double distanceToEnd;
        public double distanceFromStart;
        public double distanceToRobot;
        public boolean pathFinished;
    }

}
