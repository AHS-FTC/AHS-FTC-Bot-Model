package edu.ahs.robotics.control;

import org.junit.Test;

import static org.junit.Assert.*;

public class VectorTest {

    @Test
    public void testSideOfLine(){
        Vector v = new Vector( 1,1);
        Point robotPositon = new Point( 3,1);

        Point p1 = new Point(0,0);
        Point p2 = new Point(50,-20);
        Point p3 = new Point(3.001,1);
        Point p4 = new Point(10,-5);
        Point p5 = new Point(-10,-5);


        assertEquals(1,v.sideOfLine(p1, robotPositon));
        assertEquals(-1,v.sideOfLine(p2, robotPositon));
        assertEquals(-1,v.sideOfLine(p3, robotPositon));
        assertEquals(-1,v.sideOfLine(p4, robotPositon));
        assertEquals(1,v.sideOfLine(p5, robotPositon));

    }
}