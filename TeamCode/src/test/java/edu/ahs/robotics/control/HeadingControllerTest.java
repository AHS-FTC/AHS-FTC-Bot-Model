package edu.ahs.robotics.control;

import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;

import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.MockClock;
import edu.ahs.robotics.util.ParameterLookup;

import static org.junit.Assert.*;

public class HeadingControllerTest {

    @Before
    public void init() {
        FTCUtilities.startTestMode();
        FTCUtilities.setParameterLookup(new ParamLookup());
        FTCUtilities.setMockClock(new MockClock());
    }

    @Test
    public void getPowersRampsUp() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(0, 4));
        Path path = makePath(points);
        HeadingController controller = new HeadingController(path, 1);
        Position robotPosition = new Position(0, 0, 0);
        Velocity velocity = Velocity.makeVelocityFromSpeedDirection(0, 0);
        OdometrySystem.State state = new OdometrySystem.State(robotPosition, velocity, 0,0);
        HeadingController.Powers powers = controller.getUpdatedPowers(state);

        assertTrue(powers.leftPower > 0);
        assertTrue(powers.rightPower > 0);

        HeadingController.Powers powers2 = controller.getUpdatedPowers(state);

        assertTrue(powers2.leftPower > powers.leftPower);
        assertTrue(powers2.rightPower > powers.rightPower);
    }

    private Path makePath(ArrayList<Point> points) {
        return new Path(points, 12, 4, 36);
    }

    @Test
    public void getPowersRight() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(4, 0));
        Path path = makePath(points);
        HeadingController controller = new HeadingController(path, 1);
        Position robotPosition = new Position(0, 1, 0);
        Velocity velocity = Velocity.makeVelocityFromSpeedDirection(0, 0);
        OdometrySystem.State state = new OdometrySystem.State(robotPosition, velocity, 0,Double.POSITIVE_INFINITY);
        HeadingController.Powers powers = controller.getUpdatedPowers(state);

        assertTrue(powers.leftPower > powers.rightPower);
    }

    @Test
    public void getPowersLeft() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(4, 0));
        Path path = makePath(points);
        HeadingController controller = new HeadingController(path, 1);
        Position robotPosition = new Position(0, -1, 0);
        Velocity velocity = Velocity.makeVelocityFromSpeedDirection(0, 0);
        OdometrySystem.State state = new OdometrySystem.State(robotPosition, velocity, 0,Double.POSITIVE_INFINITY);
        HeadingController.Powers powers = controller.getUpdatedPowers(state);

        assertTrue(powers.rightPower > powers.leftPower);
    }

    @Test
    public void testPowersZeroAtEnd() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(0, 4));
        Path path = makePath(points);
        HeadingController controller = new HeadingController(path, 1);
        Position robotPosition = new Position(0, 4, 0);
        Velocity velocity = Velocity.makeVelocityFromSpeedDirection(0, 0);
        OdometrySystem.State state = new OdometrySystem.State(robotPosition, velocity, 0,0);
        HeadingController.Powers powers = controller.getUpdatedPowers(state);
        assertEquals(0, powers.leftPower, .001);
        assertEquals(0, powers.rightPower, .001);
    }

    @Test
    public void testPowersZeroPastEnd() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(0, 4));
        Path path = makePath(points);
        HeadingController controller = new HeadingController(path, 1);
        Position robotPosition = new Position(0, 5, 0);
        Velocity velocity = Velocity.makeVelocityFromSpeedDirection(0, 0);
        OdometrySystem.State state = new OdometrySystem.State(robotPosition, velocity, 0,0);
        HeadingController.Powers powers = controller.getUpdatedPowers(state);
        assertEquals(0.0, powers.leftPower, .001);
        assertEquals(0.0, powers.rightPower, .001);
    }

    @Test
    public void testPowersStayInRangeWhileRamping() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(0, 4));
        Path path = makePath(points);
        HeadingController controller = new HeadingController(path, 1);
        Position robotPosition = new Position(0, 0, 0);
        Velocity velocity = Velocity.makeVelocityFromSpeedDirection(0, 0);
        OdometrySystem.State state = new OdometrySystem.State(robotPosition, velocity, 0,0);

        double maxPower;
        do {
            HeadingController.Powers powers = controller.getUpdatedPowers(state);
            maxPower = Math.max(powers.leftPower, powers.rightPower);
        } while (maxPower < 1);
        assertEquals(1, maxPower, .001);
    }

    @Test
    public void testFuturePointWithInfiniteRadius(){
        Position p = new Position(0,0,0);
        Velocity v = Velocity.makeVelocityFromSpeedDirection(12,0);

        OdometrySystem.State state = new OdometrySystem.State(p,v,0.0,Double.POSITIVE_INFINITY);
        HeadingController controller = new HeadingController(null, 0);

        Point futurePoint = controller.getFuturePoint(state,1);

        assertEquals(new Point(12,0), futurePoint);
    }

    @Test
    public void testFuturePointWithInfiniteRadiusButBackwards(){
        Position p = new Position(0,0,Math.PI);
        Velocity v = Velocity.makeVelocityFromSpeedDirection(12,0); //direction actually doesnt matter. Wrong?

        OdometrySystem.State state = new OdometrySystem.State(p,v,0.0,Double.POSITIVE_INFINITY);
        HeadingController controller = new HeadingController(null, 0);

        Point futurePoint = controller.getFuturePoint(state,1);

        assertEquals(-12, futurePoint.x, 0.0000001);
        assertEquals(0, futurePoint.y,0.0000001);
    }

    @Test
    public void testNullMovement(){
        Position p = new Position(0,0,0);
        Velocity v = Velocity.makeVelocityFromSpeedDirection(0,0); //direction actually doesnt matter. Wrong?

        OdometrySystem.State state = new OdometrySystem.State(p,v,0.0,0);
        HeadingController controller = new HeadingController(null, 0);

        Point futurePoint = controller.getFuturePoint(state,1);

        assertEquals(0, futurePoint.x, 0.0000001);
        assertEquals(0, futurePoint.y,0.0000001);

    }

    @Test
    public void testFuturePointFullRotation(){ //travels full circumference of unit circle
        Position p = new Position(3,4,2);
        Velocity v = Velocity.makeVelocityFromSpeedDirection(2 * Math.PI,0); //direction actually doesnt matter. Wrong?

        OdometrySystem.State state = new OdometrySystem.State(p,v,0.0,1);
        HeadingController controller = new HeadingController(null, 0);

        Point futurePoint = controller.getFuturePoint(state,1);

        assertEquals(p.x, futurePoint.x,0.000001); // it goes full circle lmao
        assertEquals(p.y, futurePoint.y,0.000001);
    }

    @Test
    public void testHalfRotation(){
        Position p = new Position(-5,0,Math.PI/2);
        Velocity v = Velocity.makeVelocityFromSpeedDirection(Math.PI,0); // 10pi is circumference, time of 5

        OdometrySystem.State state = new OdometrySystem.State(p,v,0.0,-5); //going right
        HeadingController controller = new HeadingController(null, 0);

        Point futurePoint = controller.getFuturePoint(state,5);


        assertEquals(5, futurePoint.x,0.000001);
        assertEquals(0, futurePoint.y,0.000001);
    }

    @Test
    public void testQuarterTurnRight(){
        Position p = new Position(3,-6,(3*Math.PI/2));
        Velocity v = Velocity.makeVelocityFromSpeedDirection(Math.PI/2,0); // 10pi is circumference, time of 5

        OdometrySystem.State state = new OdometrySystem.State(p,v,0.0,-3);
        HeadingController controller = new HeadingController(null, 0);

        Point futurePoint = controller.getFuturePoint(state,3);

        assertEquals(0, futurePoint.x,0.000001);
        assertEquals(-9, futurePoint.y,0.000001);
    }

    @Test
    public void testQuarterTurnLeft(){
        Position p = new Position(-4,-4,0);
        Velocity v = Velocity.makeVelocityFromSpeedDirection(Math.PI/2,0); // 10pi is circumference, time of 5

        OdometrySystem.State state = new OdometrySystem.State(p,v,0.0,3);
        HeadingController controller = new HeadingController(null, 0);

        Point futurePoint = controller.getFuturePoint(state,3);

        assertEquals(-1, futurePoint.x,0.000001);
        assertEquals(-1, futurePoint.y,0.000001);
    }


    @Test
    public void testNoSpeedFuturePoint(){
        Position p = new Position(1,5,0);
        Velocity v = Velocity.makeVelocityFromSpeedDirection(0,0);

        OdometrySystem.State state = new OdometrySystem.State(p,v,0.0,50);
        HeadingController controller = new HeadingController(null, 0);

        Point futurePoint = controller.getFuturePoint(state,50);

        assertEquals(p.x, futurePoint.x,0.000001);
        assertEquals(p.y, futurePoint.y,0.000001);
    }

    @Test
    public void testPowersStayInRangeWhileTurning() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(0, 4));
        Path path = makePath(points);
        HeadingController controller = new HeadingController(path, 1);
        Position robotPosition = new Position(1, 0, 0);
        Velocity velocity = Velocity.makeVelocityFromSpeedDirection(0, 0);
        OdometrySystem.State state = new OdometrySystem.State(robotPosition, velocity, 0,0);

        double maxPower;
        do {
            HeadingController.Powers powers = controller.getUpdatedPowers(state);
            maxPower = Math.max(Math.abs(powers.leftPower), Math.abs(powers.rightPower));
        } while (maxPower < 1);
        assertEquals(1, maxPower, .001);
    }

    private class ParamLookup implements ParameterLookup {
        @Override
        public double getParameter(String name) {
            if (name.equals("p")) {

                return .001;
            }
            return 0;
        }
    }
}