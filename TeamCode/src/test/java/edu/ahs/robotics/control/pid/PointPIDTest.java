package edu.ahs.robotics.control.pid;

import org.junit.Test;

import edu.ahs.robotics.control.Point;

import static org.junit.Assert.*;

public class PointPIDTest {

    @Test
    public void testNull(){
        Point p1 = new Point(1,1);
        Point p2 = new Point(4,2);

        PointPID pid = new PointPID(0,0,0);

        PointPID.Correction correction = pid.getCorrection(p1,p2,1);

        assertEquals(0,correction.x,0);
        assertEquals(0,correction.y,0);
    }

    @Test
    public void testSame(){
        Point p = new Point(5,5);

        PointPID pid = new PointPID(1,1,1);

        PointPID.Correction correction = pid.getCorrection(p,p,1); // should be no correction when points are the same


        assertEquals(0,correction.x,0);
        assertEquals(0,correction.y,0);
    }

    @Test
    public void testProportionality(){
        Point p = new Point(1,1);
        Point pClose = new Point(1.1,1.1);
        Point pFar = new Point(2,2);

        PointPID pid = new PointPID(1,0,0);

        PointPID.Correction bigCorrection = pid.getCorrection(p,pFar,1);
        PointPID.Correction smolCorrection = pid.getCorrection(p,pClose,1);

        assertTrue(bigCorrection.x > smolCorrection.x);
        assertTrue(bigCorrection.y > smolCorrection.y);

        assertTrue(smolCorrection.x > 0); // should be greater than zero
        assertTrue(smolCorrection.y > 0);
    }

    @Test
    public void testIntegral(){
        Point p1 = new Point(2,3);
        Point p2 = new Point(6,6);

        PointPID pid = new PointPID(0,1,0);

        for(int i = 0; i < 4; i ++) {
            pid.getCorrection(p1, p2,1); //wind up integral
        }

        PointPID.Correction correction = pid.getCorrection(p1,p2,1);

        assertTrue(correction.x > 0);
        assertTrue(correction.y > 0);
    }

    @Test
    public void testDerivative(){
        Point target = new Point(10,10);
        Point far =  new Point(2,2);
        Point close =  new Point(9,9);

        PointPID pid = new PointPID(0,0,1);

        pid.getCorrection(far,target,1);

        PointPID.Correction correction = pid.getCorrection(close, target,1);

        assertTrue(correction.x < 0); //derivative should be making a negative correction
        assertTrue(correction.y < 0);
    }

}