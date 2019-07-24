package edu.ahs.robotics;

import org.junit.Test;

public class FunctionTester {
    private final double DISTANCE = 12; // in inches

    private Function function = new RampFunction(DISTANCE);

    @Test
    public void testFunction(){
        final double ITERATION_SIZE = 10;
        final double MAXTIME = 10000;// in milliseconds

        for(double i = 0; i < MAXTIME; i += ITERATION_SIZE){
            String iString = Double.toString(i);
            String distanceString = Double.toString(function.getDesiredDistance(i));
            System.out.println(iString + ", " + distanceString); //correct formatting to copypaste into Desmos as x,y pairs
        }
    }
}
