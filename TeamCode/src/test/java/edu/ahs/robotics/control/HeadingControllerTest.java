package edu.ahs.robotics.control;

import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.*;

public class HeadingControllerTest {

//    @Test
//    public void getUpdatedPowersWithDiagonal() {
//        ArrayList<Point> points = new ArrayList<>();
//        points.add(new Point(1,1));
//        points.add(new Point(3,3));
//        Path path = new Path(points);
//        HeadingController controller = new HeadingController(path, 1,1,1);
//        Position robotPosition = new Position(4,0,0);
//        Path.Location targetLocation = path.getTargetLocation(robotPosition);
//        Velocity velocity = new Velocity(12, 0);
//        HeadingController.Powers powers = controller.getUpdatedPowers(robotPosition, velocity);
//        HeadingController.Powers expectedPowers = new HeadingController.Powers(1,1);
//
//        assertEquals(expectedPowers.rightPower, powers.rightPower, .001);
//        assertEquals(expectedPowers.leftPower, powers.leftPower, .001);
//    }
//
//    @Test
//    public void getUpdatedPowersWithStraight() {
//        ArrayList<Point> points = new ArrayList<>();
//        points.add(new Point(0,0));
//        points.add(new Point(0,4));
//        Path path = new Path(points);
//        HeadingController controller = new HeadingController(path, 1,1,1);
//        Position robotPosition = new Position(0,2,0);
//        Velocity velocity = new Velocity(12, 0);
//        HeadingController.Powers powers = controller.getUpdatedPowers(robotPosition, velocity);
//        HeadingController.Powers expectedPowers = new HeadingController.Powers(1,1);
//
//        assertEquals(expectedPowers.rightPower, powers.rightPower, .001);
//        assertEquals(expectedPowers.leftPower, powers.leftPower, .001);
//    }

    @Test
    public void getPowersRampsUp() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(0, 4));
        Path path = new Path(points);
        HeadingController controller = new HeadingController(path, 12, 12, 12);
        Position robotPosition = new Position(0, 0, 0);
        Velocity velocity = Velocity.makeVelocity(0, 0);
        HeadingController.Powers powers = controller.getUpdatedPowers(robotPosition, velocity);

        assertTrue(powers.leftPower > 0);
        assertTrue(powers.rightPower > 0);

        HeadingController.Powers powers2 = controller.getUpdatedPowers(robotPosition, velocity);

        assertTrue(powers2.leftPower > powers.leftPower);
        assertTrue(powers2.rightPower > powers.rightPower);
    }

    @Test
    public void getPowersRight() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(0, 4));
        Path path = new Path(points);
        HeadingController controller = new HeadingController(path, 12, 12, 12);
        Position robotPosition = new Position(1, 0, 0);
        Velocity velocity = Velocity.makeVelocity(0, 0);
        HeadingController.Powers powers = controller.getUpdatedPowers(robotPosition, velocity);

        assertTrue(powers.rightPower > powers.leftPower);

    }

    @Test
    public void getPowersLeft() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(0, 4));
        Path path = new Path(points);
        HeadingController controller = new HeadingController(path, 12, 12, 12);
        Position robotPosition = new Position(-1, 0, 0);
        Velocity velocity = Velocity.makeVelocity(0, 0);
        HeadingController.Powers powers = controller.getUpdatedPowers(robotPosition, velocity);

        assertTrue(powers.leftPower > powers.rightPower);
    }

    @Test
    public void testPowersZeroAtEndAndOffEnd() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(0, 4));
        Path path = new Path(points);
        HeadingController controller = new HeadingController(path, 12, 12, 12);
        Position robotPosition = new Position(0, 4, 0);
        Velocity velocity = Velocity.makeVelocity(0, 0);
        HeadingController.Powers powers = controller.getUpdatedPowers(robotPosition, velocity);
        assertEquals(0, powers.leftPower, .001);
        assertEquals(0, powers.rightPower, .001);

        robotPosition = new Position(0, 3, 0);
        powers = controller.getUpdatedPowers(robotPosition, velocity);

        robotPosition = new Position(0, 5, 0);
        powers = controller.getUpdatedPowers(robotPosition, velocity);

        assertEquals(0, powers.leftPower, .001);
        assertEquals(0, powers.rightPower, .001);
    }
    @Test
    public void testPowersStayInRangeWhileRamping() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(0, 4));
        Path path = new Path(points);
        HeadingController controller = new HeadingController(path, 12, 12, 12);
        Position robotPosition = new Position(0, 0, 0);
        Velocity velocity = Velocity.makeVelocity(0, 0);
        double maxPower;
        do {
            HeadingController.Powers powers = controller.getUpdatedPowers(robotPosition, velocity);
            maxPower = Math.max(powers.leftPower, powers.rightPower);
        } while (maxPower < 1);
        assertEquals(1, maxPower, .001);
    }


    @Test
    public void testPowersStayInRangeWhileTurning() {
        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0, 0));
        points.add(new Point(0, 4));
        Path path = new Path(points);
        HeadingController controller = new HeadingController(path, 12, 12, 12);
        Position robotPosition = new Position(1, 0, 0);
        Velocity velocity = Velocity.makeVelocity(0, 0);
        double maxPower;
        do {
            HeadingController.Powers powers = controller.getUpdatedPowers(robotPosition, velocity);
            maxPower = Math.max(Math.abs(powers.leftPower), Math.abs(powers.rightPower));
        } while (maxPower < 1);
        assertEquals(1, maxPower, .001);
    }
}