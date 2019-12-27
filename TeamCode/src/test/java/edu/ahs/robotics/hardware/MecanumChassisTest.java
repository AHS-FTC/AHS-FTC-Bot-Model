package edu.ahs.robotics.hardware;

import static org.junit.Assert.*;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import edu.ahs.robotics.control.Path;
import edu.ahs.robotics.control.Point;
import edu.ahs.robotics.control.Position;
import edu.ahs.robotics.control.Velocity;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.hardware.sensors.OdometrySystemMock;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.MotorHashService;

public class MecanumChassisTest {

    private MecanumChassis mecanumChassis;
    private DcMotorMockLogger frontLeft, frontRight, backLeft, backRight;

    private void init(ArrayList<Position> positions, ArrayList<Velocity> velocities){
        FTCUtilities.startTestMode();

        OdometrySystem odometrySystem = makeOdometrySystemMock(positions, velocities);
        GearRatio driveGearRatio = new GearRatio(1,1);
        DriveUnit.Config driveConfig = new DriveUnit.Config(driveGearRatio, 3.94, MotorHashService.MotorTypes.AM_20);

        frontLeft = new DcMotorMockLogger();
        frontRight = new DcMotorMockLogger();
        backLeft = new DcMotorMockLogger();
        backRight = new DcMotorMockLogger();

        FTCUtilities.addTestMotor(frontLeft, "FL"); // add test motors here so they can get injected into mecanumChassis
        FTCUtilities.addTestMotor(frontRight, "FR");
        FTCUtilities.addTestMotor(backLeft, "BR");
        FTCUtilities.addTestMotor(backRight, "BL");

        mecanumChassis = new MecanumChassis(driveConfig, odometrySystem);
        mecanumChassis.startOdometrySystem();
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
        assertEquals(-1, Math.signum(v.forwardLeft), 0.0);
    }

    @Test
    public void testVectorConversionAt45(){

        MecanumChassis.MecanumVectors v;

        double x = 10;
        double y = 10;

        v = MecanumChassis.MecanumVectors.convertLocalVectorsToMecanumVectors(x, y);

        assertEquals(0.0, v.forwardLeft, 0.0);
        assertEquals(1, Math.signum(v.forwardRight), 0.0);
    }


    @Test
    public void testVelocityDrive(){ // standard setup that can kinda be messed around with
        double maxSpeed = 10;

        ArrayList<Velocity> velocities = new ArrayList<>();
        ArrayList<Position> positions = new ArrayList<>();

        velocities.add(Velocity.makeVelocity(maxSpeed,0));
        velocities.add(Velocity.makeVelocity(maxSpeed,0));
        velocities.add(Velocity.makeVelocity(maxSpeed,0));
        velocities.add(Velocity.makeVelocity(maxSpeed,0));
        velocities.add(Velocity.makeVelocity(maxSpeed,0)); //6 - 1

        positions.add(new Position(0,0,0));
        positions.add(new Position(20,0,0));
        positions.add(new Position(40,0,0));
        positions.add(new Position(60,0,0));
        positions.add(new Position(80,0,0));
        positions.add(new Position(100,0,0)); //6: calls for initial position, but not initial velocity.

        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0,0));
        points.add(new Point(100,0));

        Path path = new Path(points);

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

        Double[] frontLeftPowers = new Double[frontLeft.powerList.size()];
        frontLeft.powerList.toArray(frontLeftPowers);

        Double[] backLeftPowers = new Double[backLeft.powerList.size()];
        backLeft.powerList.toArray(backLeftPowers);

        Double[] frontRightPowers = new Double[frontRight.powerList.size()];
        frontRight.powerList.toArray(frontRightPowers);

        Double[] backRightPowers = new Double[backRight.powerList.size()];
        backRight.powerList.toArray(backRightPowers);

        assertArrayEquals(frontLeftPowers,backLeftPowers); // in this case all arrays should be equal
        assertArrayEquals(frontRightPowers,backRightPowers);
        assertArrayEquals(frontRightPowers, frontLeftPowers);
    }

    private OdometrySystem makeOdometrySystemMock(List<Position> positions, List<Velocity> velocities){
        return new OdometrySystemMock(positions,velocities);
    }
}