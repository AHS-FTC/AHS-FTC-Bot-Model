package edu.ahs.robotics.util;

import org.junit.Test;

import static org.junit.Assert.*;

public class FTCUtilitiesTest {

    @Test
    public void testAngleWrapper(){
        double angle = FTCMath.wrapAngle(4*Math.PI);

        assertEquals(0,angle, 0.0000001);
    }

    @Test
    public void testAngleWrapperNegative(){
        double angle = FTCMath.wrapAngle((-5*Math.PI)/2.0);

        assertEquals(-Math.PI/2, angle,0.00000001);
    }

}