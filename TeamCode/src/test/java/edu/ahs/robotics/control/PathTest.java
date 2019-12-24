package edu.ahs.robotics.control;

import org.junit.Test;

import java.util.ArrayList;

import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.Point;

import static org.junit.Assert.*;

public class PathTest {

//    @Test
//    public void testSimplePath() {
//        ArrayList<Point> points = new ArrayList<>();
//        points.add(new Point(0,0));
//        points.add(new Point(1,1));
//        points.add(new Point(3,2));
//        Path path = new Path(points);
//
//        //Test that target position is first point at totalDistance 0
//        Position targetPosition = path.getTargetLocation(0);
//        assertEquals(0, targetPosition.x, .001);
//        assertEquals(0, targetPosition.y,  .001);
//        assertEquals(45, targetPosition.getHeadingInDegrees(), .01);
//
//        //Test that target position is 1 time of point 1
//        targetPosition = path.getTargetLocation(Math.sqrt(2));
//        assertEquals(1, targetPosition.x,  .001);
//        assertEquals(1, targetPosition.y, .001);
//        assertEquals(45, targetPosition.getHeadingInDegrees(),  .01);
//
//        //Test what happens if currentTime is greater than totalDistance
//        targetPosition = path.getTargetLocation(5);
//        assertEquals(3, targetPosition.x,  .001);
//        assertEquals(2, targetPosition.y, .001);
//        assertEquals(26.5650512 , targetPosition.getHeadingInDegrees(),  .01);
//
//        //Test to see what happens if the path is longer than 2 points
//        targetPosition = path.getTargetLocation(3.95);
//        assertEquals(3, targetPosition.x,  .001);
//        assertEquals(2, targetPosition.y, .001);
//        assertEquals(26.5650512, targetPosition.getHeadingInDegrees(),  .01);
//
//    }

    @Test
    public void testPointOfIntersection() {

    }

    @Test
    public void testBoundingPointsWithSimplePath() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0,0));
        points.add(new Point(4,2));
        Path path = new Path(points);

        //Test at robot position 0
        Position robotPosition = new Position(0,0,0);
        int[] boundingPoints = path.getBoundingPoints(robotPosition);
        assertArrayEquals(new int[] {0,1}, boundingPoints);

        //Test at robot position -1
        Position robotPositionBeforeStartOfPath = new Position(-1,-3,10);
        boundingPoints = path.getBoundingPoints(robotPositionBeforeStartOfPath);
        assertArrayEquals(new int[] {0,1}, boundingPoints);

        //Test at position between bounding points
        Position robotPositionDuringPath = new Position(2,1,0);
        boundingPoints = path.getBoundingPoints(robotPositionDuringPath);
        assertArrayEquals(new int[] {0,1}, boundingPoints);

        //Test at left of path
        Position robotPositionLeftOfPath = new Position(3,-2,0);
        boundingPoints = path.getBoundingPoints(robotPositionLeftOfPath);
        assertArrayEquals(new int[] {0,1}, boundingPoints);

        //Test at right of path
        Position robotPositionRightOfPath = new Position(3,2,0);
        boundingPoints = path.getBoundingPoints(robotPositionRightOfPath);
        assertArrayEquals(new int[] {0,1}, boundingPoints);

        //Test at farthest point
        Position robotPositionAtLastPoint = new Position(4,2,0);
        boundingPoints = path.getBoundingPoints(robotPositionAtLastPoint);
        assertArrayEquals(new int[] {0,1}, boundingPoints);
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
        int[] boundingPoints = path.getBoundingPoints(robotPosition);
        assertArrayEquals(new int[] {0,1}, boundingPoints);

        //Test at robot position -1
        Position robotPositionBeforeStartOfPath = new Position(-1,-3,0);
        boundingPoints = path.getBoundingPoints(robotPositionBeforeStartOfPath);
        assertArrayEquals(new int[] {0,1}, boundingPoints);

        //Test at position between bounding points
        Position robotPositionDuringPath = new Position(2,2,0);
        boundingPoints = path.getBoundingPoints(robotPositionDuringPath);
        assertArrayEquals(new int[] {1,2}, boundingPoints);

        //Test at left of path
        Position robotPositionLeftOfPath = new Position(3,2,0);
        boundingPoints = path.getBoundingPoints(robotPositionLeftOfPath);
        assertArrayEquals(new int[] {1,2}, boundingPoints);

        //Test at right of path
        Position robotPositionRightOfPath = new Position(3,4,0);
        boundingPoints = path.getBoundingPoints(robotPositionRightOfPath);
        assertArrayEquals(new int[] {2,3}, boundingPoints);

        //Test at farthest point
        Position robotPositionAtLastPoint = new Position(6,6,0);
        boundingPoints = path.getBoundingPoints(robotPositionAtLastPoint);
        assertArrayEquals(new int[] {3,4}, boundingPoints);
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
        assertEquals(new Point(2,0), targetLocation.point);
        assertEquals(distanceFromStart, targetLocation.distanceFromStart,.001);
        assertEquals(distanceToEnd, targetLocation.distanceToEnd,.001);
        assertEquals(deltaX, targetLocation.deltaX, .001);
        assertEquals(deltaY, targetLocation.deltaY, .001);

    }

    @Test
    public void testTargetLocationWithComplexLine() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(1,1));
        points.add(new Point(3,3));
        Path path = new Path(points);

        //Test location at robot position x=2 y=0
        Position robotPosition = new Position(4,0,0);
        Path.Location targetLocation = path.getTargetLocation(robotPosition);
        double distanceFromStart = Math.sqrt(2);
        double distanceToEnd = Math.sqrt(2);
        double deltaX = 2;
        double deltaY = 2;
        assertEquals(new Point(2,2), targetLocation.point);
        assertEquals(distanceFromStart, targetLocation.distanceFromStart, .001);
        assertEquals(distanceToEnd, targetLocation.distanceToEnd, .001);
        assertEquals(deltaX, targetLocation.deltaX, .001);
        assertEquals(deltaY, targetLocation.deltaY, .001);

    }

}