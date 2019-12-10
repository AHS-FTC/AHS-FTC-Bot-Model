package edu.ahs.robotics.hardware;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import edu.ahs.robotics.hardware.sensors.IMU;
import edu.ahs.robotics.hardware.sensors.Odometer;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.Logger;

public class MecanumChassis extends Chassis {

    private static final double ROBOT_WIDTH = 333;
    public static final int MIN_TARGET_DISTANCE = 5;
    public static final double DISTANCE_PER_360 = 1001;
    private static final double LEFT_INITIAL_SHIFT = 0;
    private static final double LEFT_INITIAL_SCALE = 1.02;
    private SingleDriveUnit frontLeft;
    private SingleDriveUnit frontRight;
    private SingleDriveUnit backLeft;
    private SingleDriveUnit backRight;

    // Motor shortcuts
    private ChassisMotors.Mecanum FRONT_LEFT = ChassisMotors.Mecanum.FRONTLEFT;
    private ChassisMotors.Mecanum FRONT_RIGHT = ChassisMotors.Mecanum.FRONTRIGHT;
    private ChassisMotors.Mecanum BACK_LEFT = ChassisMotors.Mecanum.BACKLEFT;
    private ChassisMotors.Mecanum BACK_RIGHT = ChassisMotors.Mecanum.BACKRIGHT;

    private IMU imu;
    private OdometrySystem odometrySystem;

    private Odometer leftOdometer;
    private Odometer rightOdometer;


    public MecanumChassis(DriveUnit.Config driveUnitConfig) {
        super();
        frontLeft = new SingleDriveUnit(FRONT_LEFT.getDeviceName(), driveUnitConfig, false);
        frontRight = new SingleDriveUnit(FRONT_RIGHT.getDeviceName(), driveUnitConfig, true);
        backLeft = new SingleDriveUnit(BACK_LEFT.getDeviceName(), driveUnitConfig, false);
        backRight = new SingleDriveUnit(BACK_RIGHT.getDeviceName(), driveUnitConfig, true);

        leftOdometer = new Odometer("intakeL", 60, false);
        rightOdometer = new Odometer("intakeR", 60.3, true);
    }


    public MecanumChassis(DriveUnit.Config driveUnitConfig, IMU imu) {
        this(driveUnitConfig);
        this.imu = imu;
    }

//    public MecanumChassis(DriveUnit.Config driveUnitConfig, IMU imu, OdometrySystem odometrySystem, String xMotorName, String yMotorName, double odometryWheelDiameter) {
//        this(driveUnitConfig, imu);
//
//        odometrySystem = new OdometrySystem(FTCUtilities.getMotor(xMotorName), FTCUtilities.getMotor(yMotorName), imu, odometryWheelDiameter);
//    }


    /*private void motionInterpreter(PointTurn pointTurn){
        if(pointTurn.type == PointTurn.Type.RELATIVE){
            double startHeading = imu.getHeading();
            double deltaHeading = 0;
            double rampUp, rampDown;
            double power;
            double headingTo;
            double u = 0.10, d = 0.04;
            double minRampUp = .25;
            double minRampDown = .12;

            while (deltaHeading < pointTurn.targetHeading){
                deltaHeading = imu.getHeading() - startHeading;//done in the domain of delta heading because relative
                headingTo = pointTurn.targetHeading - deltaHeading;

                rampUp = Math.max(u*Math.sqrt(deltaHeading),minRampUp);
                rampDown = Math.max(d*Math.sqrt(headingTo),minRampDown);

                power = Math.min(rampUp, Math.min(rampDown, pointTurn.power));

                frontLeft.setPower(-power);
                frontRight.setPower(power);
                backLeft.setPower(-power);
                backRight.setPower(power);

                FTCUtilities.OpLogger("Delta Heading", imu.getHeading());
            }
            setPowerAll(0);
            //Logger.getInstance().writeToFile();

        } else if(pointTurn.type == PointTurn.Type.ABSOLUTE){

            /// not a thing yet --------------------------------
            double heading = imu.getHeading();
            int directionMultiplier = 1;

            if(pointTurn.targetHeading < heading) {//todo check for targetHeading directions, make sure this is right and not stupid
                directionMultiplier = -1;
            }
            while (directionMultiplier*heading < directionMultiplier*pointTurn.targetHeading){ //in theory, multiplying both sides by -1 effectively flips inequality. in theory
                heading = imu.getHeading();

                frontLeft.setPower(pointTurn.power);
                frontRight.setPower(-pointTurn.power);
                backLeft.setPower(pointTurn.power);
                backRight.setPower(-pointTurn.power);

                FTCUtilities.OpLogger("Heading", imu.getHeading());
            }
            setPowerAll(0);
        }
    }*/

    private void setPowerAll(double motorPower) {
        frontRight.setPower(motorPower);
        frontLeft.setPower(motorPower);
        backRight.setPower(motorPower);
        backLeft.setPower(motorPower);
    }

    public void stopMotors() {
        setPowerAll(0);
    }

    public void drive3Axis(double forward, double strafe, double turn) {

        double frontLeftPower = forward - strafe - turn;
        double frontRightPower = forward + strafe + turn;
        double backLeftPower = forward + strafe - turn;
        double backRightPower = forward - strafe + turn;


        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void arc(double angle, double radius, double maxPower, boolean rightTurn) {
        double minRampUp = .65;
        double minRampDown = .5;

        arc(angle, radius, maxPower, rightTurn, minRampUp, minRampDown, 5000);
    }


    public void arc(double angle, double radius, double maxPower, boolean rightTurn, double minRampUp, double minRampDown, long timeOut) {

        double leftTarget;
        double rightTarget;
        double innerTarget = angle * 2 * Math.PI * Math.abs(radius) / 360;
        double outerTarget = angle * 2 * Math.PI * (Math.abs(radius) + ROBOT_WIDTH) / 360;
        if (rightTurn) {
            leftTarget = outerTarget;
            rightTarget = innerTarget;
        } else {
            leftTarget = innerTarget;
            rightTarget = outerTarget;
        }
        FTCUtilities.sleep(1000);
        rawDrive(leftTarget, rightTarget, maxPower, minRampUp, minRampDown, .03, .01, timeOut);
    }

    public void driveStraight(double distance, double maxPower) {
        driveStraight(distance, maxPower, .55, .45, 5000);
    }
    
    public void driveStraight(double distance, double maxPower, double minRampUp, double minRampDown, long timeOut) {
        rawDrive(distance, distance, maxPower, minRampUp, minRampDown, .02, .005, timeOut);
    }

    public void pivot(double angle, double maxPower) {
        double minRampUp = .6;
        double minRampDown = .55;

        pivot(angle, maxPower, minRampUp, minRampDown, 3000);
    }

    public void pivot(double angle, double maxPower, double minRampUp, double minRampDown, long timeOut) {
        double leftTarget = (angle * DISTANCE_PER_360) / 360.0;
        double rightTarget = -1 * leftTarget;
        rawDrive(leftTarget, rightTarget, maxPower, minRampUp, minRampDown, .03, .06, timeOut);
    }


    private void rawDrive(double leftTarget, double rightTarget, double maxPower, double minRampUp, double minRampDown, double upScale, double downScale, long timeout) {

        if (maxPower < 0 || maxPower > 1.0) {
            throw new Warning("maxPower " + maxPower + " must be between 0 and 1");
        }
        if (Math.abs(leftTarget) < MIN_TARGET_DISTANCE || Math.abs(rightTarget) < MIN_TARGET_DISTANCE) {
            throw new Warning("Abs(targets) must be > MIN_TARGET_DISTANCE, " + MIN_TARGET_DISTANCE + " left target " + leftTarget + " right target " + rightTarget);
        }

        double maxTarget = Math.abs(leftTarget) > Math.abs(rightTarget) ? Math.abs(leftTarget) : Math.abs(rightTarget);

        final double correctionScale = 0.05;

        leftOdometer.reset();
        rightOdometer.reset();

        try {
            double powerLeft = inversePower(((leftTarget / maxTarget) * (minRampUp) + LEFT_INITIAL_SHIFT) * LEFT_INITIAL_SCALE);
            double powerRight = inversePower((rightTarget / maxTarget) * (minRampUp));
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < timeout) {
                double leftDistance = -leftOdometer.getDistance();
                double rightDistance = rightOdometer.getDistance();

                double leftDistanceRatio = leftDistance / leftTarget;
                double rightDistanceRatio = rightDistance / rightTarget;
                double averageDistanceRatio = (leftDistanceRatio + rightDistanceRatio) / 2;

                if (Math.abs(leftDistanceRatio) >= 1.0 && Math.abs(rightDistanceRatio) >= 1.0) { //was || before
                    break;
                }

                double maxDistance = Math.abs(leftDistance) > Math.abs(rightDistance) ? Math.abs(leftDistance) : Math.abs(rightDistance);
                double maxRemaining = maxTarget - maxDistance;
                double rampUp = Math.max(upScale * (maxDistance), minRampUp);
                double rampDown = Math.max(downScale * (maxRemaining), minRampDown); //distanceTo accounts for flip across y axis and x offset

                double targetPower = inversePower(Math.min(rampUp, Math.min(rampDown, maxPower)));

                double adjustLeft = correctionScale * (averageDistanceRatio - leftDistanceRatio);
                double adjustRight = correctionScale * (averageDistanceRatio - rightDistanceRatio);

                targetPower -= Math.max(adjustLeft, adjustRight);

                powerLeft += (adjustLeft * Math.signum(leftTarget));
                powerRight += (adjustRight * Math.signum(rightTarget));

                double higherPower = Math.max(Math.abs(powerLeft), Math.abs(powerRight));
                double rampRatio = targetPower / higherPower;

                powerLeft *= rampRatio;
                powerRight *= rampRatio;

                frontLeft.setPower(powerLeft);
                backLeft.setPower(powerLeft);
                frontRight.setPower(powerRight);
                backRight.setPower(powerRight);

                FTCUtilities.addData("power left", powerLeft);
                FTCUtilities.addData("power right", powerRight);
                FTCUtilities.addData("left ratio", leftDistanceRatio);
                FTCUtilities.addData("right ratio", rightDistanceRatio);
                FTCUtilities.addData("max remaining", maxRemaining);
                FTCUtilities.addData("leftDistance", leftDistance);
                FTCUtilities.addData("rightDistance", rightDistance);
                FTCUtilities.addData("ramp ratio", rampRatio);
                FTCUtilities.updateOpLogger();

            }
        } finally {
            setPowerAll(0);
        }
        Logger.getInstance().writeToFile();
    }

    private double inversePower(double power) {
        //   return power * Math.abs(power);
        return Math.pow(power, 3);
    }

}
