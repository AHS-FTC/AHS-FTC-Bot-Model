package edu.ahs.robotics.control;

import org.junit.Test;

import static org.junit.Assert.*;

public class PIDTest {
    @Test
    public void testDerivative(){
        PID pid = new PID(0,0,1);
        pid.getCorrection(1,10); //large error
        double correction = pid.getCorrection(9.9,10); //small error

        assertTrue(correction < 0);
    }

    @Test
    public void testNull(){
        PID pid = new PID(0,0,0);
        double correction = pid.getCorrection(0,10);

        assertEquals(0,correction,0);
    }


    @Test
    public void testIntegral(){
        PID pid = new PID(0,1,0);
        double initialCorrection = pid.getCorrection(0,1);

        pid.getCorrection(0,1);//wind up integral
        pid.getCorrection(0,1);


        double correction = pid.getCorrection(0,10);
        assertTrue(correction > initialCorrection);

    }

}