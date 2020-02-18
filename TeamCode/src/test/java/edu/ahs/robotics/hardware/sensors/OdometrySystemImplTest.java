package edu.ahs.robotics.hardware.sensors;

import org.junit.Before;
import org.junit.Test;

import edu.ahs.robotics.control.Point;
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

    @Before
    public void init(){
        FTCUtilities.startTestMode();
        FTCUtilities.setMockClock(new MockClock());
    }

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

        //assertEquals(0, state.velocity.power(),0.0);
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

        //assertTrue(state.velocity.power() > 0);
        //assertEquals(state.velocity.direction(), Math.PI / 2,0.001);
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

        //assertEquals(0,state.velocity.power(), 0.01);
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

        //assertEquals(0,state.velocity.power(), 0.01);
    }

    @Test
    public void strafe1Foot(){
        double[] x1Inputs = {0,0,0,0}; //OdometrySystemImpl references once upon init - starting with zero is a good idea
        double[] x2Inputs = {0,0,0,0};
        double[] yInputs = {0,-2,-4,-12};
        init(x1Inputs, x2Inputs, yInputs);

        for(int i = 0; i < x1Inputs.length - 1; i++){ //-1 accounts for the initial call to the resetEncoders() method
            odometrySystem.updatePosition();
        }

        assertEquals(12, odometrySystem.getState().position.x, .01);
    }

    @Test
    public void strafeAndDrive1Foot(){
        double[] x1Inputs = {0,2,4,12}; //OdometrySystemImpl references once upon init - starting with zero is a good idea
        double[] x2Inputs = {0,2,4,12};
        double[] yInputs = {0,-2,-4,-12};
        init(x1Inputs, x2Inputs, yInputs);

        for(int i = 0; i < x1Inputs.length - 1; i++){ //-1 accounts for the initial call to the resetEncoders() method
            odometrySystem.updatePosition();
        }

        OdometrySystem.State state = odometrySystem.getState();

        assertEquals(12, state.position.x, .01);
        assertEquals(12, state.position.y, .01);
      
        //assertEquals(Math.PI/4, state.velocity.direction(),0.001);
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

        //assertEquals(0,state.velocity.power(), 0.01);
    }

    @Test //THIS TEST USES NO DY. REINSTATABLE FOR EMERGENCIES
    public void testFullTurn(){
        double distanceBetweenWheels = 12.0; //24pi id, 48pi od

        double[] xLInputs = {0, 6*Math.PI ,  12*Math.PI, 18*Math.PI, 24*Math.PI}; //OdometrySystemImpl references once upon init - starting with zero is a good idea
        double[] xRInputs = {0, 12*Math.PI , 24*Math.PI, 36*Math.PI, 48*Math.PI}; //Driving a circle of radius 2*distance between wheels on outside
        double[] yInputs = {0,0,0,0,0};

        OdometerMock xR = new OdometerMock(xRInputs);
        OdometerMock xL = new OdometerMock(xLInputs);
        OdometerMock y = new OdometerMock(yInputs);

        OdometrySystemImpl odometrySystem = new OdometrySystemImpl(xR, xL,y, 0, distanceBetweenWheels);
        odometrySystem.setPosition(0,0,0);

        for(int i = 0; i < xLInputs.length; i++){ //-1 accounts for the initial call to the resetEncoders() method
            odometrySystem.updatePosition();
        }

        OdometrySystem.State state = odometrySystem.getState();

        assertEquals(0, state.position.x,0.0001);
        assertEquals(0, state.position.y,0.0001);
    }

    @Test
    public void realisticArcTest(){ //Made to emulate arc auto
        double[] xLInputs = {0,12.93943474195, 25.8788694839,38.81830422585, 51.7577389678}; //OdometrySystemImpl references once upon init - starting with zero is a good idea
        double[] xRInputs = {0,18.4764917939, 36.9529835878, 55.4294753817, 73.9059671756};
        double[] yInputs = {0,0,0,0};

        OdometerMock xR = new OdometerMock(xRInputs);
        OdometerMock xL = new OdometerMock(xLInputs);
        OdometerMock y = new OdometerMock(yInputs);

        OdometrySystemImpl odometrySystem = new OdometrySystemImpl(xR, xL,y, 0, 14.1);

        for(int i = 0; i < xLInputs.length; i++){ //-1 accounts for the initial call to the resetEncoders() method
            odometrySystem.updatePosition();
        }

        OdometrySystem.State state = odometrySystem.getState();

        assertEquals(Math.PI/2, state.position.heading,0.000001);
        assertEquals(40, state.position.x,0.000001);
        assertEquals(40, state.position.y,0.000001);
        //assertEquals(40, state.travelRadius,0.000001);
    }

    @Test
    public void dynamicCircleTest(){
        double radius = 40;
        double distanceBetweenWheels = 14.1;
        int segments = 15;
        double percentOfCircle = .25;//1 is full circle

        double innerArcRadius = radius - (distanceBetweenWheels/2.0);
        double outerArcRadius = radius + (distanceBetweenWheels/2.0);

        double innerArc = 2 * innerArcRadius * Math.PI * percentOfCircle;
        double outerArc = 2 * outerArcRadius * Math.PI * percentOfCircle;

        Point centerOfArc = new Point(0,0);

        double[] xLInputs = new double[segments];
        double[] xRInputs = new double[segments];
        double[] yInputs =  new double[segments];


        //create inputs
        for(int i = 0; i < segments; i++){
            xLInputs[i] = (i * innerArc)/ (double)segments; //fractions of the total arc, 1/5, 2/5, 3/5... etc
            xRInputs[i] = (i * outerArc)/ (double)segments;
        }
        OdometerMock xR = new OdometerMock(xRInputs);
        OdometerMock xL = new OdometerMock(xLInputs);
        OdometerMock y = new OdometerMock(yInputs);
        OdometrySystemImpl odometrySystem = new OdometrySystemImpl(xR, xL,y, 0, distanceBetweenWheels);

        odometrySystem.setPosition(0,-40, 0);
        for(int i = 0; i < xLInputs.length; i++){ //-1 accounts for the initial call to the resetEncoders() method
            odometrySystem.updatePosition();

            OdometrySystem.State state = odometrySystem.getState();
            double x2 = Math.pow(state.position.x, 2);
            double y2 = Math.pow(state.position.y, 2);
            double r2 = Math.pow(radius, 2);

            assertEquals(r2, x2+y2, 0.001);


           // double distanceFromCenter = centerOfArc.distanceTo(state.position);

            //assertEquals(radius, distanceFromCenter, 0.000001);
        }

    }

//    @Test
//    public void testVelocity(){
//        double[] x1Inputs = {0,2,4,6,8,10,12,14,16,18,20,22,24}; //OdometrySystemImpl references once upon init - starting with zero is a good idea
//        double[] x2Inputs = {0,2,4,6,8,10,12,14,16,18,20,22,24};
//        double[] yInputs = {0,0,0,0,0,0,0,0,0,0,0,0,0};
//
//        //---------------- custom init
//        FTCUtilities.startTestMode();
//        MockClock clock = new MockClock(MockClock.Mode.CUSTOM_TIME_STEP);
//        clock.setTimeStep(1000); //advance by a second to make velocity calculations easier
//        FTCUtilities.setMockClock(clock);
//
//        OdometerMock x1 = new OdometerMock(x1Inputs);
//        OdometerMock x2 = new OdometerMock(x2Inputs);
//        OdometerMock y = new OdometerMock(yInputs);
//
//        odometrySystem = new OdometrySystemImpl(x1, x2, y, .1, 12);
//        odometrySystem.setPosition(0,0,0); // note different heading
//        odometrySystem.resetEncoders();
//
//        FTCUtilities.getCurrentTimeMillis();
//
//        Velocity velocity;
//        //---------------- custom init
//
//        for(int i = 0; i < x1Inputs.length - 1; i++){
//            odometrySystem.updatePosition();
//
//
//            velocity = odometrySystem.getState().velocity;
//
//            assertEquals(2,velocity.power(), 0.0001);
//            assertEquals(0,velocity.direction(), 0.0001);
//        }
//
//    }


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