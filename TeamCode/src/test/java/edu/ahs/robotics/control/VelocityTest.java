package edu.ahs.robotics.control;

import org.junit.Test;

import static org.junit.Assert.*;

public class VelocityTest {

    @Test
    public void testDxDyWithVerticalLine() {
        Velocity v = Velocity.makeVelocityFromDxDy(0,1);

        assertEquals(Math.PI/2,v.direction,0.0);
        assertEquals(1,v.speed,0.0);
    }

    @Test
    public void testDxDyWithHorizontalLine() {
        Velocity v = Velocity.makeVelocityFromDxDy(3,0);

        assertEquals(0,v.direction,0.0);
        assertEquals(3,v.speed,0.0);
    }

    @Test
    public void testDxDyWithNegativeLine() {
        Velocity v = Velocity.makeVelocityFromDxDy(-1,-1);

        assertEquals(Math.toRadians(225),v.direction,0.0000001);
        assertEquals(Math.sqrt(2),v.speed,0.0000001);
    }
}