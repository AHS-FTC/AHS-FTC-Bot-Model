package edu.ahs.robotics.control;

import static org.junit.Assert.*;

import org.junit.Test;

public class VelocityTest {

    @Test
    public void testDxDyWithVerticalLine() {
        Velocity v = Velocity.makeVelocity(0,1);

        assertEquals(Math.PI/2,v.direction(),0.0);
        assertEquals(1,v.speed(),0.0);
    }

    @Test
    public void testDxDyWithHorizontalLine() {
        Velocity v = Velocity.makeVelocity(3,0);

        assertEquals(0,v.direction(),0.0);
        assertEquals(3,v.speed(),0.0);
    }

    @Test
    public void testDxDyWithNegativeLine() {
        Velocity v = Velocity.makeVelocity(-1,-1);

        assertEquals(Math.toRadians(225),v.direction(),0.0000001);
        assertEquals(Math.sqrt(2),v.speed(),0.0000001);
    }

    @Test
    public void testVelocityScaling(){
        Velocity v = Velocity.makeVelocity(3,4); // 3,4,5 triangle
        v.scaleMagnitude(25);

        assertEquals(25,v.speed(),0.0000001);
        assertEquals(15,v.dx,0.0000001);
        assertEquals(20,v.dy,0.0000001);
    }

    @Test
    public void testNegativeVelocityScaling(){
        Velocity v = Velocity.makeVelocity(3,4); // 3,4,5 triangle
        v.scaleMagnitude(-5);

        assertEquals(5,v.speed(),0.0000001);
        assertEquals(-3,v.dx,0.0000001);
        assertEquals(-4,v.dy,0.0000001);
    }


}