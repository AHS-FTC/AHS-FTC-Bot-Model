package edu.ahs.robotics.control;

import org.junit.Test;

import java.util.ArrayList;

import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.Point;

import static org.junit.Assert.*;

public class PathTest {

    @Test
    public void testBoundingPointsWithSimplePath() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0,0));
        points.add(new Point(4,2));
        Path path = new Path(points);


        //Test at robot position 0
        Position robotPosition = new Position(0,0,0);
        int boundingPoint = path.getBoundingPoints(robotPosition);
        assertEquals(0, boundingPoint);

        //Test at robot position -1
        Position robotPositionBeforeStartOfPath = new Position(-1,-3,10);
        boundingPoint = path.getBoundingPoints(robotPositionBeforeStartOfPath);
        assertEquals(0, boundingPoint);

        //Test at position between bounding points
        Position robotPositionDuringPath = new Position(2,1,0);
        boundingPoint = path.getBoundingPoints(robotPositionDuringPath);
        assertEquals(0, boundingPoint);

        //Test at left of path
        Position robotPositionLeftOfPath = new Position(3,-2,0);
        boundingPoint = path.getBoundingPoints(robotPositionLeftOfPath);
        assertEquals(0, boundingPoint);

        //Test at right of path
        Position robotPositionRightOfPath = new Position(3,2,0);
        boundingPoint = path.getBoundingPoints(robotPositionRightOfPath);
        assertEquals(0, boundingPoint);

        //Test at farthest point
        Position robotPositionAtLastPoint = new Position(4,2,0);
        boundingPoint = path.getBoundingPoints(robotPositionAtLastPoint);
        assertEquals(1, boundingPoint);
    }

    @Test
    public void testBoundingPointsWithComplexPath() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0,0));
        points.add(new Point(1,1));
        points.add(new Point(3,3));
        points.add(new Point(5,3));
        points.add(new Point(6,6));
        Path path = new Path(points);

        //Test at robot position 0
        Position robotPosition = new Position(0,0,0);
        int boundingPoints = path.getBoundingPoints(robotPosition);
        assertEquals(0, boundingPoints);

        //Test at robot position -1
        Position robotPositionBeforeStartOfPath = new Position(-1,-3,0);
        boundingPoints = path.getBoundingPoints(robotPositionBeforeStartOfPath);
        assertEquals(0, boundingPoints);

        //Test at position between bounding points
        Position robotPositionDuringPath = new Position(2,2,0);
        boundingPoints = path.getBoundingPoints(robotPositionDuringPath);
        assertEquals(1, boundingPoints);

        //Test at left of path
        Position robotPositionLeftOfPath = new Position(3,2,0);
        boundingPoints = path.getBoundingPoints(robotPositionLeftOfPath);
        assertEquals(1, boundingPoints);

        //Test at right of path
        Position robotPositionRightOfPath = new Position(3,4,0);
        boundingPoints = path.getBoundingPoints(robotPositionRightOfPath);
        assertEquals(2, boundingPoints);

        //Test at farthest point
        Position robotPositionAtLastPoint = new Position(6,6,0);
        boundingPoints = path.getBoundingPoints(robotPositionAtLastPoint);
        assertEquals(4, boundingPoints);
    }

    @Test
    public void testTargetLocationWithSimpleLine() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0,0));
        points.add(new Point(4,0));
        Path path = new Path(points);

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
        Path path = new Path(points);

        //Test location at robot position x=2 y=2
        Position robotPosition = new Position(4,0,0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);
        double distanceFromStart = Math.sqrt(2);
        double distanceToEnd = Math.sqrt(2);
        double deltaX = 2;
        double deltaY = 2;
        assertEquals(new Point(2,2), targetLocation.closestPoint);
        assertEquals(distanceFromStart, targetLocation.distanceFromStart, .001);
        assertEquals(distanceToEnd, targetLocation.distanceToEnd, .001);
        assertEquals(deltaX, targetLocation.pathDeltaX, .001);
        assertEquals(deltaY, targetLocation.pathDeltaY, .001);
        assertEquals(Math.sqrt(8), targetLocation.distanceToRobot, .001);
        assertFalse(targetLocation.pathFinished);

        robotPosition = new Position(0,4, 0);
        targetLocation = path.getTargetLocation(robotPosition);
        assertEquals(-(Math.sqrt(8)), targetLocation.distanceToRobot, .001);
        assertFalse(targetLocation.pathFinished);

        //Test location at robot position over end of path
        robotPosition = new Position(4, 4, 0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertTrue(targetLocation.pathFinished);
    }

    @Test
    public void testGetDistanceToRobotBottomLeftQuadrant() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(-1,-1));
        points.add(new Point(-3,-3));
        Path path = new Path(points);

        //Test location at robot position x = -2, y = -2, left of line
        Position robotPosition = new Position(0,-4,0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(-(Math.sqrt(8)), targetLocation.distanceToRobot, .001);
        assertFalse(targetLocation.pathFinished);

        //Test location at robot position x = -2, y = -2, right of line
        robotPosition = new Position(-4,0,0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(Math.sqrt(8), targetLocation.distanceToRobot, .001);
        assertFalse(targetLocation.pathFinished);

        //Test location at robot position over end of path
        robotPosition = new Position(-4, -4, 0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertTrue(targetLocation.pathFinished);
    }

    @Test
    public void testGetDistanceToRobotBottomRightQuadrant() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(1,-1));
        points.add(new Point(3,-3));
        Path path = new Path(points);

        //Test location at robot position x = 2, y = -2, left of line
        Position robotPosition = new Position(4,0,0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(-(Math.sqrt(8)), targetLocation.distanceToRobot, .001);
        assertFalse(targetLocation.pathFinished);

        //Test location at robot position x = 2, y = -2, right of line
        robotPosition = new Position(0,-4,0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(Math.sqrt(8), targetLocation.distanceToRobot, .001);
        assertFalse(targetLocation.pathFinished);

        //Test location at robot position over end of path
        robotPosition = new Position(4, -4, 0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertTrue(targetLocation.pathFinished);
    }

    @Test
    public void testGetDistanceToRobotTopLeftQuadrant() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(-1,1));
        points.add(new Point(-3,3));
        Path path = new Path(points);

        //Test location at robot position x = 2, y = -2, left of line
        Position robotPosition = new Position(-4,0,0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(-(Math.sqrt(8)), targetLocation.distanceToRobot, .001);
        assertFalse(targetLocation.pathFinished);

        //Test location at robot position x = 2, y = -2, right of line
        robotPosition = new Position(0,4,0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(Math.sqrt(8), targetLocation.distanceToRobot, .001);
        assertFalse(targetLocation.pathFinished);

        //Test location at robot position over end of path
        robotPosition = new Position(-4, 4, 0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertTrue(targetLocation.pathFinished);
    }

    @Test
    public void testGetDistanceToRobotNot45DegreeLine() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(-1,1));
        points.add(new Point(-3,5));
        Path path = new Path(points);

        //Test location at robot position x = -2, y = 3, left of line
        Position robotPosition = new Position(-4,2,0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(-(Math.sqrt(5)), targetLocation.distanceToRobot, .001);
        assertFalse(targetLocation.pathFinished);

        //Test location at robot position x = -2, y = 3, right of line
        robotPosition = new Position(0,4,0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(Math.sqrt(5), targetLocation.distanceToRobot, .001);
        assertFalse(targetLocation.pathFinished);

        //Test location at robot position over end of path
        robotPosition = new Position(-4, 6, 0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertTrue(targetLocation.pathFinished);
    }

    @Test
    public void testGetDistanceToRobotHorizontalLine() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(-1,0));
        points.add(new Point(3,0));
        Path path = new Path(points);

        //Test location at robot position x = 2, y = 0, left of line
        Position robotPosition = new Position(2,2,0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(-(Math.sqrt(4)), targetLocation.distanceToRobot, .001);
        assertFalse(targetLocation.pathFinished);

        //Test location at robot position x = 2, y = 0, right of line
        robotPosition = new Position(2,-2,0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(Math.sqrt(4), targetLocation.distanceToRobot, .001);
        assertFalse(targetLocation.pathFinished);

        //Test location at robot position over end of path
        robotPosition = new Position(4, 0, 0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertTrue(targetLocation.pathFinished);
    }

    @Test
    public void testGetDistanceToRobotVerticalLine() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0,-1));
        points.add(new Point(0,3));
        Path path = new Path(points);

        //Test location at robot position x = 0, y = 2, left of line
        Position robotPosition = new Position(-2,2,0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(-(Math.sqrt(4)), targetLocation.distanceToRobot, .001);
        assertFalse(targetLocation.pathFinished);

        //Test location at robot position x = 2, y = 0, right of line
        robotPosition = new Position(2,2,0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertEquals(Math.sqrt(4), targetLocation.distanceToRobot, .001);
        assertFalse(targetLocation.pathFinished);

        //Test location at robot position over end of path
        robotPosition = new Position(0, 4, 0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertTrue(targetLocation.pathFinished);
    }

    @Test
    public void testPathFinished() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(0, 4));
        Path path = new Path(points);

        //Test location at robot position during path
        Position robotPosition = new Position(0, 2, 0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);

        assertFalse(targetLocation.pathFinished);

        //Test location at robot position at end of path
        robotPosition = new Position(0, 4, 0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertTrue(targetLocation.pathFinished);

        //Test location at robot position over end of path
        robotPosition = new Position(0, 5, 0);
        targetLocation = path.getTargetLocation(robotPosition);

        assertTrue(targetLocation.pathFinished);
    }
}