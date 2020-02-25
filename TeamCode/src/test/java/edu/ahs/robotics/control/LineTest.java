package edu.ahs.robotics.control;

import org.junit.Test;

import static org.junit.Assert.*;

public class LineTest {
    private static Point ORIGIN = new Point(0,0);

    @Test
    public void testAxis(){
        Line xAxis = new Line(0,1,0);
        Line yAxis = new Line(1,0,0);
        Point intersection = xAxis.findIntersection(yAxis);

        assertEquals(intersection.x, 0 ,0);
        assertEquals(intersection.y, 0 ,0);

        Line maybeYAxis = xAxis.getPerpLineAtPoint(new Point(0,0));

        assertEquals(0, maybeYAxis.b,0);
        assertEquals(0, maybeYAxis.c,0);

    }

    @Test
    public void testFlippedPoints(){ // if you canFlip the points on input you should receive the same line
        Point p1 = new Point(3, -4);
        Point p2 = new Point(5, 6);

        Line line1 = new Line(p1, p2);
        Line line2 = new Line(p2, p1);

        assertEquals(line1.findSlope(), line2.findSlope(),0); // line coefficients may not be the same, but slope should be
    }

    @Test
    public void testVerticalLine(){
        Point p1 = new Point(3, -5);
        Point p2 = new Point(3, 5);

        Line line = new Line(p1, p2);
        Line perp = line.getPerpLineAtPoint(ORIGIN);
        Point intersection = line.findIntersection(perp);

        assertEquals(0, perp.a,0); // should be horizontal

        assertEquals(0,perp.findSlope(),0);
        assertEquals(0, intersection.y,0); //int at (3,0)
        assertEquals(3, intersection.x, 0);

    }

    @Test
    public void testYEqualsX(){
        Point p = new Point(1,1);
        Line yEqX = new Line(ORIGIN, p);

        assertEquals(1, yEqX.findSlope(),0);

        Line perp = yEqX.getPerpLineAtPoint(ORIGIN);
        assertEquals(-1, perp.findSlope(),0);

        Point intersection = yEqX.findIntersection(perp);

        assertEquals(0, intersection.x,0);
        assertEquals(0, intersection.y,0);

    }

    @Test
    public void testParallelLines(){
        boolean error = false;
        Point p1 = new Point(3, -4);
        Point p2 = new Point(5, 6);

        Line line = new Line(p1, p2);

        try{
            line.findIntersection(line); //this should throw an error, because a line is parallel with itself.
        } catch (Exception e) {
            error = true;
        }
        assertTrue(error);
    }

    @Test
    public void testNearestPointWhenAlreadyOnLine(){
        Point p = new Point(1,1); // line of y = x
        Line yEqX = new Line(ORIGIN, p);

        Position pos = new Position(3,3,700000000); //heading doesn't matter

        Point closestPoint = yEqX.getClosestPointOnLine(pos);

        assertEquals(3, closestPoint.x, 0);
        assertEquals(3, closestPoint.y, 0);

    }

    @Test
    public void testNearestPoint(){
        //Math: https://www.desmos.com/calculator/a6sdkkzogq

        Line l = new Line(1, -1, 1);
        Position pos = new Position(1.6, 1,69); //heading doesn't matter, again

        Point closestPoint = l.getClosestPointOnLine(pos);

        assertEquals(1.8, closestPoint.x,0);
        assertEquals(.8, closestPoint.y,0);

        assertEquals(1, l.findSlope(), 0);

    }


}