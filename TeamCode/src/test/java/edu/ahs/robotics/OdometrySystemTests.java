package edu.ahs.robotics;

import org.junit.Test;
import static org.junit.Assert.*;
import org.junit.runners.model.TestClass;

import edu.ahs.robotics.hardware.sensors.Odometer;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;


public class OdometrySystemTests {
    OdometrySystem odometrySystem;

    private void init(double[] y1Inputs, double[] y2Inputs, double[] xInputs){
        FTCUtilities.startTestMode();

        OdometerMock y1 = new OdometerMock(y1Inputs);
        OdometerMock y2 = new OdometerMock(y2Inputs);
        OdometerMock x = new OdometerMock(xInputs);

        odometrySystem = new OdometrySystem(y1, y2, x, .1, 12);
    }

    @Test
    public void driveForward1Foot(){
        double[] y1Inputs = {0,2,8,12}; //OdometrySystem references once upon init - starting with zero is a good idea
        double[] y2Inputs = {0,2,8,12};
        double[] xInputs = {0,0,0,0};
        init(y1Inputs, y2Inputs, xInputs);

        //odometrySystem.resetPosition(0,0,0);

        assertEquals(12, odometrySystem.getPosition().y, .1);
    }

    @Test
    public void turn90Degrees(){
        double[] y1Inputs = {0,2,8,12};
        double[] y2Inputs = {0,2,8,12};
        double[] xInputs = {0,0,0,0};
        init(y1Inputs,y2Inputs,xInputs);

        assertEquals(90, odometrySystem.getPosition().heading, .1);
    }

}
