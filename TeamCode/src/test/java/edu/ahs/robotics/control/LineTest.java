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

        assertSame(intersection, ORIGIN);
    }

}