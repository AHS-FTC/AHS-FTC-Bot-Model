package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static org.junit.Assert.*;

import org.junit.Ignore;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.Point;
import edu.ahs.robotics.control.Position;
import edu.ahs.robotics.control.Vector;
import edu.ahs.robotics.control.Velocity;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.hardware.sensors.OdometrySystemMock;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.MockClock;
import edu.ahs.robotics.util.MotorHashService;

public class MecanumChassisTest {

    private MecanumChassis mecanumChassis;
    private DcMotorMockLogger frontLeft, frontRight, backLeft, backRight;
    private Double[] frontLeftPowers, frontRightPowers, backLeftPowers, backRightPowers;

    private void init(ArrayList<Position> positions, ArrayList<Velocity> velocities){
        FTCUtilities.startTestMode();

        MockClock mc = new MockClock();
        FTCUtilities.setMockClock(mc);

        OdometrySystem odometrySystem = makeOdometrySystemMock(positions, velocities);
        GearRatio driveGearRatio = new GearRatio(1,1);
        DriveUnit.Config driveConfig = new DriveUnit.Config(driveGearRatio, 3.94, MotorHashService.MotorTypes.AM_20);

        frontLeft = new DcMotorMockLogger();
        frontRight = new DcMotorMockLogger();
        backLeft = new DcMotorMockLogger();
        backRight = new DcMotorMockLogger();

        FTCUtilities.addTestMotor(frontLeft, "FL"); // add test motors here so they can get injected into mecanumChassis
        FTCUtilities.addTestMotor(frontRight, "FR");
        FTCUtilities.addTestMotor(backLeft, "BL");
        FTCUtilities.addTestMotor(backRight, "BR");

        mecanumChassis = new MecanumChassis(driveConfig, odometrySystem);
        mecanumChassis.startOdometrySystem();
    }

    @Test
    public void testDriveTowardsPointSimple(){
        Position position = new Position(0,0,0);
        Point targetPoint = new Point(60,0);
        init(null,null);

        MecanumChassis.DriveCommand command = mecanumChassis.getDriveTowardsPointCommands(targetPoint,1,.5, 12,position,0);

        Vector v = command.driveVector;
        assertEquals(1, v.x, 0.0);
        assertEquals(0, v.y, 0.0);
        assertEquals(0, command.turnOutput,0.0);
    }

    @Test
    public void testDriveTowardsPointAt45(){
        Position position = new Position(-5,0,Math.PI);
        Point targetPoint = new Point(-8,-3);
        init(null, null);

        MecanumChassis.DriveCommand command = mecanumChassis.getDriveTowardsPointCommands(targetPoint,1,.5, 1,position,0);

        Vector v = command.driveVector;

        assertEquals(v.x, v.y, 0.000000001);
        System.out.println(command.turnOutput);
        assertTrue(command.turnOutput > 0.0);
    }

    @Test
    public void testDriveTowardsPointComplex(){
        Position position = new Position(-3,5,Math.PI/2);
        Point targetPoint = new Point(12,0); //use desmos to graph these points if necessary
        init(null,null);

        MecanumChassis.DriveCommand command = mecanumChassis.getDriveTowardsPointCommands(targetPoint,1,.5, 1,position,0);

        Vector v = command.driveVector;

        assertEquals(-1.0/3.0,v.x/-v.y,0.00000001); //since vector is local, local x is global y and local y is negative global x. then do a slope calculation
        assertTrue(command.turnOutput < 0);
    }

    @Test
    public void testDriveTowardsPointInReverse(){
        Position position = new Position(0,0,Math.PI);
        Point targetPoint = new Point(12,3);
        init(null,null);

        MecanumChassis.DriveCommand command = mecanumChassis.getDriveTowardsPointCommands(targetPoint,1,.5, 1,position,Math.PI);

        Vector v = command.driveVector;

        assertEquals((1.0/4.0),v.y/v.x,0.00000001);
        assertTrue(command.turnOutput < 0);
    }

    @Test
    public void testVectorConversionZeros(){
        MecanumChassis.MecanumVectors v;

        double x = 0;
        double y = 0;

        v = MecanumChassis.MecanumVectors.convertLocalVectorsToMecanumVectors(x,y);

        assertEquals(0,v.forwardLeft,0.0);
        assertEquals(0,v.forwardRight, 0.0);
    }

    @Test
    public void testVectorConversionStraightForward(){
        MecanumChassis.MecanumVectors v;

        double x = 10;
        double y = 0;

        v = MecanumChassis.MecanumVectors.convertLocalVectorsToMecanumVectors(x,y);

        assertEquals(v.forwardLeft, v.forwardRight, 0.0);
        assertEquals(1, Math.signum(v.forwardLeft), 0.0);
    }

    @Test
    public void testVectorConversionStrafe() {
        MecanumChassis.MecanumVectors v;

        double x = 0;
        double y = 10;

        v = MecanumChassis.MecanumVectors.convertLocalVectorsToMecanumVectors(x, y);

        assertEquals(v.forwardLeft, - v.forwardRight, 0.0);
        assertEquals(-1, Math.signum(v.forwardRight), 0.0);
    }

    @Test
    public void testVectorConversionAt45(){

        MecanumChassis.MecanumVectors v;

        double x = 10;
        double y = 10;

        v = MecanumChassis.MecanumVectors.convertLocalVectorsToMecanumVectors(x, y);

        assertEquals(0.0, v.forwardRight, 0.0);
        assertEquals(1, Math.signum(v.forwardLeft), 0.0);
    }

    @Test
    @Ignore
    public void testPointPID(){
        Point targetPoint = new Point(0,0);
        long timeout =  100000000; // don't timeout

        ArrayList<Position> positions = new ArrayList<>();

        positions.add(new Position(10,10,0));
        positions.add(new Position(9,9,0));
        positions.add(new Position(8,8,0));
        positions.add(new Position(7,7,0));
        positions.add(new Position(6,6,0));
        positions.add(new Position(5,5,0));
        positions.add(new Position(4,4,0));
        positions.add(new Position(3,3,0));
        positions.add(new Position(2,2,0));
        positions.add(new Position(1,1,0));
        positions.add(new Position(0,0,0));

        init(positions,null);

        mecanumChassis.goToPointWithPID(targetPoint,timeout);

        FTCUtilities.OpLogger("FL", frontLeft.powerList);
        FTCUtilities.OpLogger("FR", frontRight.powerList);
        FTCUtilities.OpLogger("BL", backLeft.powerList);
        FTCUtilities.OpLogger("BR", backRight.powerList);


        fillPowerLists();

        Double[] emptyList = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; //10

        assertArrayEquals(emptyList,frontLeftPowers);
        assertArrayEquals(emptyList,backRightPowers);

        assertArrayEquals(backLeftPowers, frontRightPowers);

        //FTCUtilities.OpLogger("FL", frontLeftPowers);

    }


    @Test
    @Ignore
    public void testVelocityDrive(){ // standard setup that can kinda be messed around with
        double maxSpeed = 10;

        ArrayList<Velocity> velocities = new ArrayList<>();
        ArrayList<Position> positions = new ArrayList<>();

        velocities.add(Velocity.makeVelocityFromSpeedDirection(maxSpeed,0));
        velocities.add(Velocity.makeVelocityFromSpeedDirection(maxSpeed,0));
        velocities.add(Velocity.makeVelocityFromSpeedDirection(maxSpeed,0));
        velocities.add(Velocity.makeVelocityFromSpeedDirection(maxSpeed,0));
        velocities.add(Velocity.makeVelocityFromSpeedDirection(maxSpeed,0)); //6 - 1

        positions.add(new Position(0,0,0));
        positions.add(new Position(20,0,0));
        positions.add(new Position(40,0,0));
        positions.add(new Position(60,0,0));
        positions.add(new Position(80,0,0));
        positions.add(new Position(100,0,0)); //6: calls for initial position, but not initial velocity.

        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0,0));
        points.add(new Point(100,0));

        Path path = new Path(points, 12, 4, 36, false);

        init(positions, velocities);

        mecanumChassis.velocityDrive(path, maxSpeed);

        FTCUtilities.OpLogger("FL", frontLeft.powerList);
        FTCUtilities.OpLogger("FR", frontRight.powerList);
        FTCUtilities.OpLogger("BL", backLeft.powerList);
        FTCUtilities.OpLogger("BR", backRight.powerList);

        assertEquals(5,frontLeft.powerList.size()); // 5 because of initial state check
        assertEquals(5,frontRight.powerList.size());
        assertEquals(5,backLeft.powerList.size());
        assertEquals(5,backRight.powerList.size());

        fillPowerLists();

        assertArrayEquals(frontLeftPowers,backLeftPowers); // in this case all arrays should be equal
        assertArrayEquals(frontRightPowers,backRightPowers);
        assertArrayEquals(frontRightPowers, frontLeftPowers);
    }

    private OdometrySystem makeOdometrySystemMock(List<Position> positions, List<Velocity> velocities){
        return new OdometrySystemMock(positions,velocities);
    }

    private void fillPowerLists (){
        frontLeftPowers = new Double[frontLeft.powerList.size()];
        frontLeft.powerList.toArray(frontLeftPowers);

        backLeftPowers = new Double[backLeft.powerList.size()];
        backLeft.powerList.toArray(backLeftPowers);

        frontRightPowers = new Double[frontRight.powerList.size()];
        frontRight.powerList.toArray(frontRightPowers);

        backRightPowers = new Double[backRight.powerList.size()];
        backRight.powerList.toArray(backRightPowers);
    }
}