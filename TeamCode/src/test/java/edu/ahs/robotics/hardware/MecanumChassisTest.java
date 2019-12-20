package edu.ahs.robotics.hardware;

import static org.junit.Assert.*;
import org.junit.Test;



public class MecanumChassisTest {

    @Test
    public void testVectorConversionZeros(){
        MecanumChassis.MecanumVectors v;

        double x = 0;
        double y = 0;

        v = MecanumChassis.MecanumVectors.convertLocalVectorsToMecanumVectors(x,y);

        assertEquals(0,v.forwardLeft,0.0);
        assertEquals(0,v.forwardRight, 0.0);
    }

    @Test
    public void testVectorConversionStraightForward(){
        MecanumChassis.MecanumVectors v;

        double x = 0;
        double y = 10;

        v = MecanumChassis.MecanumVectors.convertLocalVectorsToMecanumVectors(x,y);

        assertEquals(v.forwardLeft, v.forwardRight, 0.0);
        assertEquals(1, Math.signum(v.forwardLeft), 0.0);
    }

    @Test
    public void testVectorConversionStrafe() {
        MecanumChassis.MecanumVectors v;

        double x = 10;
        double y = 0;

        v = MecanumChassis.MecanumVectors.convertLocalVectorsToMecanumVectors(x, y);

        assertEquals(v.forwardLeft, - v.forwardRight, 0.0);
        assertEquals(-1, Math.signum(v.forwardLeft), 0.0);
    }

    @Test
    public void testVectorConversionAt45(){

        MecanumChassis.MecanumVectors v;

        double x = 10;
        double y = 10;

        v = MecanumChassis.MecanumVectors.convertLocalVectorsToMecanumVectors(x, y);

        assertEquals(0.0, v.forwardLeft, 0.0);
        assertEquals(1, Math.signum(v.forwardRight), 0.0);
    }

}