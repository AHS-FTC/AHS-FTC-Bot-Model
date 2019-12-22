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
//        Position targetPosition = path.getTargetPosition(0);
//        assertEquals(0, targetPosition.x, .001);
//        assertEquals(0, targetPosition.y,  .001);
//        assertEquals(45, targetPosition.getHeadingInDegrees(), .01);
//
//        //Test that target position is 1 time of point 1
//        targetPosition = path.getTargetPosition(Math.sqrt(2));
//        assertEquals(1, targetPosition.x,  .001);
//        assertEquals(1, targetPosition.y, .001);
//        assertEquals(45, targetPosition.getHeadingInDegrees(),  .01);
//
//        //Test what happens if currentTime is greater than totalDistance
//        targetPosition = path.getTargetPosition(5);
//        assertEquals(3, targetPosition.x,  .001);
//        assertEquals(2, targetPosition.y, .001);
//        assertEquals(26.5650512 , targetPosition.getHeadingInDegrees(),  .01);
//
//        //Test to see what happens if the path is longer than 2 points
//        targetPosition = path.getTargetPosition(3.95);
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
        points.add(new Point(1,1));
        Path path = new Path(points);

        //Test at robot position 0
        Position robotPosition = new Position(0,0,0);
        int[] boundingPoints = path.getBoundingPoints(robotPosition);
        assertArrayEquals(new int[] {0,1}, boundingPoints);

        //Test
    }
}