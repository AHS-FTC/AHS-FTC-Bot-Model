package edu.ahs.robotics.control;

import org.junit.Test;

import static org.junit.Assert.*;


/**
 * some tests.
 * @author Yerboi Alex Appleby
 */
public class XYHeadingPIDTest {

    private XYHeadingPID pid;
    private static final Position ORIGIN = new Position(0,0,0);

    private void init(XYHeadingPID.Config config){
        pid = new XYHeadingPID(config);
    }

    @Test
    public void testZeroCase(){
        XYHeadingPID.Config config = new XYHeadingPID.Config();
        config.setHeadingParameters(1,1,-1);
        config.setXParameters(1,1,-1);
        config.setYParameters(1,1,-1);

        init(config);
        XYHeadingPID.Correction correction = pid.getCorrection(ORIGIN, ORIGIN);

        assertEquals(correction.x, 0, 0); // when target position = current position, there shouldn't be a correction
        assertEquals(correction.y, 0,0);
        assertEquals(correction.heading, 0,0);
    }

    @Test
    public void testProportionality(){
        XYHeadingPID.Config config = new XYHeadingPID.Config();
        config.setHeadingParameters(1,1,-1);
        config.setXParameters(1,1,-1);
        config.setYParameters(1,1,-1);

        init(config);
        Position targetPositionShort = new Position(10,10, 0);
        Position targetPositionLong = new Position(100,100, 0);


        XYHeadingPID.Correction smallCorrection = pid.getCorrection(ORIGIN, targetPositionShort);
        XYHeadingPID.Correction bigCorrection = pid.getCorrection(ORIGIN, targetPositionLong);

        assertTrue(smallCorrection.x < bigCorrection.x);
        assertTrue(smallCorrection.y < bigCorrection.y);
    }
    @Test
    public void testProportionalityOffOrigin(){
        XYHeadingPID.Config config = new XYHeadingPID.Config();
        config.setHeadingParameters(1,1,-1);
        config.setXParameters(1,1,-1);
        config.setYParameters(1,1,-1);

        init(config);
        Position robotPosition = new Position( 5, 5, 0);

        Position targetPositionShort = new Position(10,10, 0);
        Position targetPositionLong = new Position(100,100, 0);

        XYHeadingPID.Correction smallCorrection = pid.getCorrection(robotPosition, targetPositionShort);
        XYHeadingPID.Correction bigCorrection = pid.getCorrection(robotPosition, targetPositionLong);

        assertTrue(smallCorrection.x < bigCorrection.x);
        assertTrue(smallCorrection.y < bigCorrection.y);
    }

    @Test
    public void testIntegral(){
        XYHeadingPID.Config config = new XYHeadingPID.Config();
        config.setHeadingParameters(0,1,0);
        config.setXParameters(0,1,0);
        config.setYParameters(0,1,0);

        init(config);
        Position targetPosition = new Position(10,10, Math.PI);

        for(int i = 0; i < 10; i++){ //wind up the integral term
            pid.getCorrection(ORIGIN, targetPosition);
        }

        XYHeadingPID.Correction correction = pid.getCorrection(ORIGIN, targetPosition);

        assertTrue(correction.x > 1); //1 is arbitrary, this should just be big
        assertTrue(correction.y > 1);
        assertTrue(correction.heading > 1);
    }
    @Test
    public void testAllZeroCoefficients(){
        XYHeadingPID.Config config = new XYHeadingPID.Config();
        config.setHeadingParameters(0,0,0);
        config.setXParameters(0,0,0);
        config.setYParameters(0,0,0);

        init(config);

        Position targetPosition = new Position(10,10, Math.PI);

        XYHeadingPID.Correction correction = pid.getCorrection(ORIGIN, targetPosition);

        assertEquals(0, correction.x,0);
        assertEquals(0, correction.y,0);
        assertEquals(0, correction.heading,0);
    }

    @Test
    public void testDerivative(){
        XYHeadingPID.Config config = new XYHeadingPID.Config();
        config.setHeadingParameters(0,0,-1);
        config.setXParameters(0,0,-1);
        config.setYParameters(0,0,-1);

        init(config);

        Position close = new Position(0.1, 0.1, 0.1); //goes from being very far (high error)
        Position far = new Position(10, 10, 10); //   to being very close (low error)
        //assuming actual is below target, the high derivative tells us we need to slow down real quick, so we make a negative adjustment

        pid.getCorrection(ORIGIN, far); //intentionally ramp up the derivative
        XYHeadingPID.Correction correction = pid.getCorrection(ORIGIN, close); // this should make a negative adj to counteract steep slope

        assertTrue(correction.x < 0);
        assertTrue(correction.y < 0);
        assertTrue(correction.heading < 0);
    }

    @Test
    /**
     * Compare the PID of a robot at origin to that of one at 0,0,pi/2 rads.
     */
    public void testCorrectionsAt90Degrees(){
        XYHeadingPID.Config config = new XYHeadingPID.Config();
        config.setHeadingParameters(0,1,0);
        config.setXParameters(0,1,0);
        config.setYParameters(0,1,0);

        XYHeadingPID pidAtOrigin = new XYHeadingPID(config); //probably a good idea to use separate systems here, as integral and derivative terms would mix otherwise
        XYHeadingPID pidAt90 = new XYHeadingPID(config);

        Position targetPosition = new Position( 5, 5, 0);

        Position robotAt90Position = new Position(0,0,Math.PI/2);

        XYHeadingPID.Correction atOriginCorrection = pidAtOrigin.getCorrection(ORIGIN,targetPosition);
        XYHeadingPID.Correction at90Correction = pidAt90.getCorrection(robotAt90Position,targetPosition);

        assertEquals(atOriginCorrection.x, at90Correction.y, 0.001);
        assertEquals(atOriginCorrection.y, at90Correction.x, 0.001);
    }

    @Test
    /**
     * Compare the PID of a robot at origin to that of one at 0,0,pi rads.
     */
    public void testCorrectionsAt180Degrees(){
        XYHeadingPID.Config config = new XYHeadingPID.Config();
        config.setHeadingParameters(0,1,0);
        config.setXParameters(0,1,0);
        config.setYParameters(0,1,0);

        XYHeadingPID pidAtOrigin = new XYHeadingPID(config); //probably a good idea to use separate systems here, as integral and derivative terms would mix otherwise
        XYHeadingPID pidAt180 = new XYHeadingPID(config);

        Position targetPosition = new Position( 5, 5, 0);

        Position robotAt180Position = new Position(0,0,Math.PI);

        XYHeadingPID.Correction atOriginCorrection = pidAtOrigin.getCorrection(ORIGIN,targetPosition);
        XYHeadingPID.Correction at180Correction = pidAt180.getCorrection(robotAt180Position,targetPosition);

        assertEquals(atOriginCorrection.x, -at180Correction.x, 0.001);
        assertEquals(atOriginCorrection.y, -at180Correction.y, 0.001);

    }

    @Test
    public void testOnlyRotationalCorrection(){ // off of origin because why not
        XYHeadingPID.Config config = new XYHeadingPID.Config();
        config.setHeadingParameters(1,1,-1);
        config.setXParameters(1,1,-1);
        config.setYParameters(1,1,-1);

        init(config);
        Position robotPosition = new Position( -5, 10, (-3*Math.PI)/2);
        Position targetPosition = new Position(-5,10, -2*(Math.PI));

        XYHeadingPID.Correction correction = pid.getCorrection(robotPosition, targetPosition);

        assertTrue(correction.heading < 0);
        assertEquals(correction.x, 0 ,0);
        assertEquals(correction.y, 0 ,0);
    }
}