package edu.ahs.robotics.hardware.sensors;

import org.junit.Test;

import edu.ahs.robotics.hardware.DcMotorMockEncoder;
import edu.ahs.robotics.util.ftc.FTCUtilities;

import static org.junit.Assert.*;

public class OdometerImplTest {

    @Test
    public void testDiscritized(){
        FTCUtilities.startTestMode();

        int listSize = 100;

        int[] encoderVals = new int[listSize];

        for (int i = 0; i < listSize; i++) {
            encoderVals[i] = i;
        }

        System.out.println(encoderVals.toString());

        DcMotorMockEncoder mockEncoder = new DcMotorMockEncoder(encoderVals);

        FTCUtilities.addTestMotor(mockEncoder, "m");

        OdometerImpl odometerImpl = new OdometerImpl("m",3.5,false,4000);

        double lastDistance = -1;

        for (int i = 0; i < listSize; i++) {
            double distance = odometerImpl.getDistance();
            assertNotEquals(lastDistance, distance,0);
            lastDistance = distance;
        }
    }
}