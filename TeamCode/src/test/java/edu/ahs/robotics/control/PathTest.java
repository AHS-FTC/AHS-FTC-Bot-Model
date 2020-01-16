package edu.ahs.robotics.control;

import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.*;

public class PathTest {

    @Test
    public void testBoundingPointsWithSimplePath() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0,0));
        points.add(new Point(4,2));
        Path path = makePath(points);


        //Test at robot position 0
        Position robotPosition = new Position(0,0,0);
        path.updateFirstBoundingPoint(robotPosition);
        assertEquals(0, path.iFirstBoundingPoint);

        //Test at robot position -1
        Position robotPositionBeforeStartOfPath = new Position(-1,-3,10);
        path.updateFirstBoundingPoint(robotPositionBeforeStartOfPath);
        assertEquals(0, path.iFirstBoundingPoint);

        //Test at position between bounding points
        Position robotPositionDuringPath = new Position(2,1,0);
        path.updateFirstBoundingPoint(robotPositionDuringPath);
        assertEquals(0, path.iFirstBoundingPoint);

        //Test at left of path
        Position robotPositionLeftOfPath = new Position(3,-2,0);
        path.updateFirstBoundingPoint(robotPositionLeftOfPath);
        assertEquals(0, path.iFirstBoundingPoint);

        //Test at right of path
        Position robotPositionRightOfPath = new Position(3,2,0);
        path.updateFirstBoundingPoint(robotPositionRightOfPath);
        assertEquals(0, path.iFirstBoundingPoint);

        //Test at farthest point
        Position robotPositionAtLastPoint = new Position(4,2,0);
        path.updateFirstBoundingPoint(robotPositionAtLastPoint);
        assertEquals(0, path.iFirstBoundingPoint);
    }

    @Test
    public void testBoundingPointsWithComplexPath() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0,0));
        points.add(new Point(1,1));
        points.add(new Point(3,3));
        points.add(new Point(5,3));
        points.add(new Point(6,6));
        Path path = makePath(points);

        //Test at robot position 0
        Position robotPosition = new Position(0,0,0);
        path.updateFirstBoundingPoint(robotPosition);
        assertEquals(0, path.iFirstBoundingPoint);

        //Test at robot position -1
        Position robotPositionBeforeStartOfPath = new Position(-1,-3,0);
        path.updateFirstBoundingPoint(robotPositionBeforeStartOfPath);
        assertEquals(0, path.iFirstBoundingPoint);

        //Test at position between bounding points
        Position robotPositionDuringPath = new Position(2,2,0);
        path.updateFirstBoundingPoint(robotPositionDuringPath);
        assertEquals(1, path.iFirstBoundingPoint);

        //Test at left of path
        Position robotPositionLeftOfPath = new Position(3,2,0);
        path.updateFirstBoundingPoint(robotPositionLeftOfPath);
        assertEquals(1, path.iFirstBoundingPoint);

        //Test at right of path
        Position robotPositionRightOfPath = new Position(3,4,0);
        path.updateFirstBoundingPoint(robotPositionRightOfPath);
        assertEquals(2, path.iFirstBoundingPoint);

        //Test at farthest point
        Position robotPositionAtLastPoint = new Position(6,6,0);
        path.updateFirstBoundingPoint(robotPositionAtLastPoint);
        assertEquals(3, path.iFirstBoundingPoint);
    }

    @Test
    public void testTargetLocationWithSimpleLine() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0,0));
        points.add(new Point(4,0));
        Path path = makePath(points);

        //Test location at robot position 2
        Position robotPosition = new Position(2,0,0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);
        double distanceFromStart = 2;
        double distanceToEnd = 2;
        double deltaX = 4;
        double deltaY = 0;
        assertEquals(new Point(2,0), targetLocation.closestPoint);
        assertEquals(distanceFromStart, targetLocation.distanceFromStart,.001);
        assertEquals(distanceToEnd, targetLocation.distanceToEnd,.001);
        assertEquals(deltaX, targetLocation.pathDeltaX, .001);
        assertEquals(deltaY, targetLocation.pathDeltaY, .001);
        assertEquals(0, targetLocation.distanceToRobot, .001);

    }

    @Test
    public void testTargetLocationWithComplexLineAndDistanceToRobotTopRightQuadrant() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(1,1));
        points.add(new Point(3,3));
        points.add(new Point(6,3));
        Path path = makePath(points);

        //Test lookAhead at start
        Position robotPosition = new Position(1, 1, 0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        //Test location at robot position x=2 y=2
        robotPosition = new Position(4,0,0);
        targetLocation = path.getTargetLocation(robotPosition);
        assertEquals(new Point(2,2), targetLocation.closestPoint);
        assertEquals(Math.sqrt(2), targetLocation.distanceFromStart, .001);
        assertEquals(Math.sqrt(2) + 3, targetLocation.distanceToEnd, .001);
        assertEquals(2, targetLocation.pathDeltaX, .001);
        assertEquals(2, targetLocation.pathDeltaY, .001);
        assertEquals(Math.sqrt(8), targetLocation.distanceToRobot, .001);
        assertFalse(path.isFinished(robotPosition));

        robotPosition = new Position(0,4, 0);
        targetLocation = path.getTargetLocation(robotPosition);
        assertEquals(-(Math.sqrt(8)), targetLocation.distanceToRobot, .001);
        assertFalse(path.isFinished(robotPosition));

        //Test location at robot position over end of path
        robotPosition = new Position(6, 3, 0);

        assertTrue(path.isFinished(robotPosition));
    }

    @Test
    public void testGetDistanceToRobotBottomLeftQuadrant() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(-1,-1));
        points.add(new Point(-3,-3));
        points.add(new Point(-3,-6));
        Path path = makePath(points);

        //Test lookAhead at start
        Position robotPosition = new Position(-1, -1, 0);
        Path.Location targetLocation;

        //Test location at robot position x = -2, y = -2, left of line
        robotPosition = new Position(0,-4,0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(-(Math.sqrt(8)), targetLocation.distanceToRobot, .001);
        assertFalse(path.isFinished(robotPosition));

        //Test location at robot position x = -2, y = -2, right of line
        robotPosition = new Position(-4,0,0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(Math.sqrt(8), targetLocation.distanceToRobot, .001);
        assertFalse(path.isFinished(robotPosition));

        //Test location at robot position over end of path
        robotPosition = new Position(-4, -7, 0);

        assertTrue(path.isFinished(robotPosition));
    }

    private Path makePath(ArrayList<Point> points) {
        return new Path(points, 12, 4, 36);
    }

    @Test
    public void testGetDistanceToRobotBottomRightQuadrant() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(1,-1));
        points.add(new Point(3,-3));
        Path path = makePath(points);

        //Test location at robot position x = 2, y = -2, left of line
        Position robotPosition = new Position(4,0,0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(-(Math.sqrt(8)), targetLocation.distanceToRobot, .001);
        assertFalse(path.isFinished(robotPosition));

        //Test location at robot position x = 2, y = -2, right of line
        robotPosition = new Position(0,-4,0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(Math.sqrt(8), targetLocation.distanceToRobot, .001);
        assertFalse(path.isFinished(robotPosition));

        //Test location at robot position over end of path
        robotPosition = new Position(4, -4, 0);

        assertTrue(path.isFinished(robotPosition));
    }

    @Test
    public void testGetDistanceToRobotTopLeftQuadrant() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(-1,1));
        points.add(new Point(-3,3));
        Path path = makePath(points);

        //Test location at robot position x = 2, y = -2, left of line
        Position robotPosition = new Position(-4,0,0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(-(Math.sqrt(8)), targetLocation.distanceToRobot, .001);
        assertFalse(path.isFinished(robotPosition));

        //Test location at robot position x = 2, y = -2, right of line
        robotPosition = new Position(0,4,0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(Math.sqrt(8), targetLocation.distanceToRobot, .001);
        assertFalse(path.isFinished(robotPosition));

        //Test location at robot position over end of path
        robotPosition = new Position(-4, 4, 0);

        assertTrue(path.isFinished(robotPosition));
    }

    @Test
    public void testGetDistanceToRobotNot45DegreeLine() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(-1,1));
        points.add(new Point(-3,5));
        Path path = makePath(points);

        //Test location at robot position x = -2, y = 3, left of line
        Position robotPosition = new Position(-4,2,0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(-(Math.sqrt(5)), targetLocation.distanceToRobot, .001);
        assertFalse(path.isFinished(robotPosition));

        //Test location at robot position x = -2, y = 3, right of line
        robotPosition = new Position(0,4,0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(Math.sqrt(5), targetLocation.distanceToRobot, .001);
        assertFalse(path.isFinished(robotPosition));

        //Test location at robot position over end of path
        robotPosition = new Position(-4, 6, 0);

        assertTrue(path.isFinished(robotPosition));
    }

    @Test
    public void testGetDistanceToRobotHorizontalLine() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(-1,0));
        points.add(new Point(3,0));
        Path path = makePath(points);

        //Test location at robot position x = 2, y = 0, left of line
        Position robotPosition = new Position(2,2,0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(-(Math.sqrt(4)), targetLocation.distanceToRobot, .001);
        assertFalse(path.isFinished(robotPosition));

        //Test location at robot position x = 2, y = 0, right of line
        robotPosition = new Position(2,-2,0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(Math.sqrt(4), targetLocation.distanceToRobot, .001);
        assertFalse(path.isFinished(robotPosition));

        //Test location at robot position over end of path
        robotPosition = new Position(4, 0, 0);

        assertTrue(path.isFinished(robotPosition));
    }

    @Test
    public void testGetDistanceToRobotVerticalLine() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0,-1));
        points.add(new Point(0,3));
        Path path = makePath(points);

        //Test location at robot position x = 0, y = 2, left of line
        Position robotPosition = new Position(-2,2,0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(-(Math.sqrt(4)), targetLocation.distanceToRobot, .001);
        assertFalse(path.isFinished(robotPosition));

        //Test location at robot position x = 2, y = 0, right of line
        robotPosition = new Position(2,2,0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(Math.sqrt(4), targetLocation.distanceToRobot, .001);
        assertFalse(path.isFinished(robotPosition));

        //Test location at robot position over end of path
        robotPosition = new Position(0, 4, 0);

        assertTrue(path.isFinished(robotPosition));
    }

    @Test
    public void testPathFinished() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(0, 4));
        Path path = makePath(points);

        //Test location at robot position during path
        Position robotPosition = new Position(0, 2, 0);

        assertFalse(path.isFinished(robotPosition));

        //Test location at robot position at end of path
        robotPosition = new Position(0, 4, 0);

        assertTrue(path.isFinished(robotPosition));

        //Test location at robot position over end of path
        robotPosition = new Position(0, 5, 0);

        assertTrue(path.isFinished(robotPosition));
    }

    @Test
    public void testGetTargetLocationPastEndOfPath() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(0, 4));
        Path path = makePath(points);

        //Test location at robot position over end of path
        Position robotPosition = new Position(0, 5, 0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(new Point(0 , 5), targetLocation.closestPoint);

    }

}