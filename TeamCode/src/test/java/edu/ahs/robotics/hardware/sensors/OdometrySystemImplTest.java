package edu.ahs.robotics.hardware.sensors;

import org.junit.Test;

import edu.ahs.robotics.control.Velocity;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.MockClock;

import static org.junit.Assert.*;

/**
 * Just some test methods for OdometrySystemImpl.
 * Needs a few more complex tests, they are just difficult to write without a robot.
 *
 * Note that we <b>do not</b> use odometrySystem.start() to ensure all tests are completely deterministic.
 * We iterate through odometrySystem.updatePosition() instead.
 * @author Alex Appleby
 */
public class OdometrySystemImplTest {
    private OdometrySystemImpl odometrySystem;

    private void init(double[] x1Inputs, double[] x2Inputs, double[] yInputs){
        FTCUtilities.startTestMode();
        MockClock clock = new MockClock(MockClock.Mode.ADVANCE_BY_10_MILLIS);
        FTCUtilities.setMockClock(clock);

        OdometerMock x1 = new OdometerMock(x1Inputs);
        OdometerMock x2 = new OdometerMock(x2Inputs);
        OdometerMock y = new OdometerMock(yInputs);

        odometrySystem = new OdometrySystemImpl(x1, x2, y, .1, 12);
        odometrySystem.setPosition(0,0,Math.PI/2);
        odometrySystem.resetEncoders();

        FTCUtilities.getCurrentTimeMillis();//advance slightly
    }

    @Test
    public void testNullMovement(){
        double[] x1Inputs = {0,0,0,0}; //OdometrySystemImpl references once upon init - starting with zero is a good idea
        double[] x2Inputs = {0,0,0,0};
        double[] yInputs = {0,0,0,0};
        init(x1Inputs, x2Inputs, yInputs);

        for(int i = 0; i < x1Inputs.length - 1; i++){ //-1 accounts for the odometrySystem constructor getting initial position
            odometrySystem.updatePosition();
        }

        OdometrySystem.State state = odometrySystem.getState();

        assertEquals(0, state.position.y, 0.0);
        assertEquals(0, state.position.heading, Math.PI/2);
        assertEquals(0, state.position.x, 0.0);

        assertEquals(0, state.velocity.speed(),0.0);
        assertEquals(0,state.travelCurvature,0.0);
    }

    @Test
    public void driveForward1Foot(){
        double[] x1Inputs = {0,2,8,12}; //OdometrySystemImpl references once upon init - starting with zero is a good idea
        double[] x2Inputs = {0,2,8,12};
        double[] yInputs = {0,0,0,0};
        init(x1Inputs, x2Inputs, yInputs);

        for(int i = 0; i < x1Inputs.length - 1; i++){ //-1 accounts for the initial call to the resetEncoders() method
            odometrySystem.updatePosition();
        }

        OdometrySystem.State state = odometrySystem.getState();

        assertEquals(12, state.position.y, .01);
        assertEquals(Math.PI/2, state.position.heading, .01);
        assertEquals(0, state.position.x, .01);

        assertTrue(state.velocity.speed() > 0);
        assertEquals(state.velocity.direction(), Math.PI / 2,0.001);
        assertEquals(0,state.travelCurvature,0.0);
    }

    @Test
    public void testImperfectStrafe(){
        double[] x1Inputs = {0,0.001,0.002,0.003}; //slightly turns, thus imperfect
        double[] x2Inputs = {0,0,0,0}; //OdometrySystemImpl references once upon init - starting with zero is a good idea
        double[] yInputs = {0,6,8,12};

        FTCUtilities.startTestMode();
        MockClock clock = new MockClock(MockClock.Mode.ADVANCE_BY_10_MILLIS);
        FTCUtilities.setMockClock(clock);

        OdometerMock x1 = new OdometerMock(x1Inputs);
        OdometerMock x2 = new OdometerMock(x2Inputs);
        OdometerMock y = new OdometerMock(yInputs);

        odometrySystem = new OdometrySystemImpl(x1, x2, y, .1, 12);
        odometrySystem.setPosition(0,0,0); // at true origin
        odometrySystem.resetEncoders();

        for(int i = 0; i < x1Inputs.length - 1; i++){ //-1 accounts for the initial call to the resetEncoders() method
            odometrySystem.updatePosition();
        }

        OdometrySystem.State state = odometrySystem.getState();


        assertEquals(12,state.position.y,0.1);
        assertEquals(0,state.position.x,0.1);
        assertEquals(0,state.travelCurvature,0.5);
    }

    @Test
    public void driveLeft1Foot(){
        double[] x1Inputs = {0,2,8,12}; //OdometrySystemImpl references once upon init - starting with zero is a good idea
        double[] x2Inputs = {0,2,8,12};
        double[] yInputs = {0,0,0,0};
        init(x1Inputs, x2Inputs, yInputs);
        odometrySystem.setPosition(0,0,0);

        for(int i = 0; i < x1Inputs.length - 1; i++){ //-1 accounts for the initial call to the resetEncoders() method
            odometrySystem.updatePosition();
        }

        OdometrySystem.State state = odometrySystem.getState();

        assertEquals(12, state.position.x, .001);
        assertEquals(0, state.position.y, .001);
    }

    @Test
    public void turn90Degrees(){
        double[] x1Inputs = {0,0,9.424775};
        double[] x2Inputs = {0,0,-9.424775};
        double[] yInputs = {0,0,9};
        init(x1Inputs,x2Inputs,yInputs);

        for(int i = 0; i < x1Inputs.length - 1; i++){ //-1 accounts for the initial call to the resetEncoders() method
            odometrySystem.updatePosition();
        }

        OdometrySystem.State state = odometrySystem.getState();

        assertEquals(Math.PI, state.position.heading, .01);
        assertEquals(0, state.position.x, .01);
        assertEquals(0, state.position.y, .01);

        assertEquals(0,state.velocity.speed(), 0.01);
        assertEquals(Double.POSITIVE_INFINITY,state.travelCurvature,0.0);
    }

    @Test
    public void turnNegative90Degrees(){
        double[] x1Inputs = {0,0,-9.424775};
        double[] x2Inputs = {0,0,9.424775};
        double[] yInputs = {0,0,-9};
        init(x1Inputs,x2Inputs,yInputs);

        for(int i = 0; i < x1Inputs.length - 1; i++){ //-1 accounts for the initial call to the resetEncoders() method
            odometrySystem.updatePosition();
        }

        OdometrySystem.State state = odometrySystem.getState();

        assertEquals(0, state.position.heading, .01);
        assertEquals(0, state.position.x, .01);
        assertEquals(0, state.position.y, .01);

        assertEquals(0,state.velocity.speed(), 0.01);
        assertEquals(Double.NEGATIVE_INFINITY,state.travelCurvature,0.0);
    }

    @Test
    public void strafe1Foot(){
        double[] x1Inputs = {0,0,0,0}; //OdometrySystemImpl references once upon init - starting with zero is a good idea
        double[] x2Inputs = {0,0,0,0};
        double[] yInputs = {0,2,4,12};
        init(x1Inputs, x2Inputs, yInputs);

        for(int i = 0; i < x1Inputs.length - 1; i++){ //-1 accounts for the initial call to the resetEncoders() method
            odometrySystem.updatePosition();
        }

        assertEquals(12, odometrySystem.getState().position.x, .01);
        assertEquals(0,odometrySystem.getState().travelCurvature,0.5);
    }

    @Test
    public void strafeAndDrive1Foot(){
        double[] x1Inputs = {0,2,4,12}; //OdometrySystemImpl references once upon init - starting with zero is a good idea
        double[] x2Inputs = {0,2,4,12};
        double[] yInputs = {0,2,4,12};
        init(x1Inputs, x2Inputs, yInputs);

        for(int i = 0; i < x1Inputs.length - 1; i++){ //-1 accounts for the initial call to the resetEncoders() method
            odometrySystem.updatePosition();
        }

        OdometrySystem.State state = odometrySystem.getState();

        assertEquals(12, state.position.x, .01);
        assertEquals(12, state.position.y, .01);
      
        assertEquals(Math.PI/4, state.velocity.direction(),0.001);
        assertEquals(0,state.travelCurvature,0.0);
    }

    @Test
    public void turn500Degrees(){
        double[] x1Inputs = {0,0,52.3598611111}; //OdometrySystemImpl references once upon init - starting with zero is a good idea
        double[] x2Inputs = {0,0,-52.3598611111};
        double[] yInputs = {0,0,50};
        init(x1Inputs, x2Inputs, yInputs);

        for(int i = 0; i < x1Inputs.length - 1; i++){ //-1 accounts for the initial call to the resetEncoders() method
            odometrySystem.updatePosition();
        }

        OdometrySystem.State state = odometrySystem.getState();

        assertEquals(Math.toRadians(500) + Math.PI/2, state.position.heading, .01);
        assertEquals(0, state.position.x, .01);
        assertEquals(0, state.position.y, .01);

        assertEquals(0,state.velocity.speed(), 0.01);
        assertEquals(Double.POSITIVE_INFINITY,state.travelCurvature,0.0);

    }

    @Test
    public void testVelocity(){
        double[] x1Inputs = {0,2,4,6,8,10,12,14,16,18,20,22,24}; //OdometrySystemImpl references once upon init - starting with zero is a good idea
        double[] x2Inputs = {0,2,4,6,8,10,12,14,16,18,20,22,24};
        double[] yInputs = {0,0,0,0,0,0,0,0,0,0,0,0,0};

        //---------------- custom init
        FTCUtilities.startTestMode();
        MockClock clock = new MockClock(MockClock.Mode.CUSTOM_TIME_STEP);
        clock.setTimeStep(1000); //advance by a second to make velocity calculations easier
        FTCUtilities.setMockClock(clock);

        OdometerMock x1 = new OdometerMock(x1Inputs);
        OdometerMock x2 = new OdometerMock(x2Inputs);
        OdometerMock y = new OdometerMock(yInputs);

        odometrySystem = new OdometrySystemImpl(x1, x2, y, .1, 12);
        odometrySystem.setPosition(0,0,0); // note different heading
        odometrySystem.resetEncoders();

        FTCUtilities.getCurrentTimeMillis();

        Velocity velocity;
        //---------------- custom init

        for(int i = 0; i < x1Inputs.length - 1; i++){
            odometrySystem.updatePosition();
            velocity = odometrySystem.getState().velocity;

            assertEquals(2,velocity.speed(), 0.0001);
            assertEquals(0,velocity.direction(), 0.0001);
        }

    }


//    @Test
//    public void pointArc90Deg(){
//        double[] y1Inputs = {0,6,18.84955}; //OdometrySystemImpl references once upon init - starting with zero is a good idea
//        double[] y2Inputs = {0,0,0};
//        double[] xInputs = {0,4.5,9};
//        init(y1Inputs, y2Inputs, xInputs);
//
//        for(int i = 0; i < y1Inputs.length - 1; i++){ //-1 accounts for the initial call to the resetEncoders() method
//            odometrySystem.updatePosition();
//        }
//
//        assertEquals(90, odometrySystem.getPosition().heading, .1);
//        //assertEquals(-10.392, odometrySystem.getPosition().x, .05);
//        assertEquals((7.63944)/2, odometrySystem.getPosition().y, .1);
//    }

}