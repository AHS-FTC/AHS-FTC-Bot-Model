package edu.ahs.robotics.control;

import org.junit.Test;

import static org.junit.Assert.*;

public class PositionTest {
    private static final double PI = Math.PI;

    @Test
    public void testAngleToVertical(){
        Position origin = new Position(0,0,0);
        Position onYAxis = new Position(0,5,0);

        double up90 = origin.angleTo(onYAxis);
        double down90 = onYAxis.angleTo(origin);

        assertEquals(PI/2,up90, 0.00001);
        assertEquals((3*PI)/2, down90, 0.00001);
    }

    @Test
    public void testAngleTo45(){
        Position origin = new Position(0,0,0);
        Position oneOne = new Position(1,1,0);

        double up45 = origin.angleTo(oneOne);
        double down45 = oneOne.angleTo(origin);

        assertEquals(PI/4, up45, 0.00001);
        assertEquals((5*PI)/4, down45, 0.00001);
    }

    @Test
    public void testAngleToHorizontal(){
        Position origin = new Position(0,0,0);
        Position onXAxis = new Position(3,0,0);

        double left = origin.angleTo(onXAxis);
        double right = onXAxis.angleTo(origin);

        assertEquals(0, left, 0.00001);
        assertEquals(PI, right, 0.00001);
    }

}