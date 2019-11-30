package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.Map;

import edu.ahs.robotics.hardware.sensors.Odometer;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.hardware.sensors.IMU;
import edu.ahs.robotics.util.Logger;
import edu.ahs.robotics.hardware.sensors.OdometrySystem;

public class MecanumChassis extends Chassis {

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
        frontLeft = new SingleDriveUnit(FRONT_LEFT.getDeviceName(),driveUnitConfig,false);
        frontRight = new SingleDriveUnit(FRONT_RIGHT.getDeviceName(), driveUnitConfig, true);
        backLeft = new SingleDriveUnit(BACK_LEFT.getDeviceName(), driveUnitConfig, false);
        backRight = new SingleDriveUnit(BACK_RIGHT.getDeviceName(), driveUnitConfig, true);

        leftOdometer = new Odometer("intakeL", 60.2967 , false);
        rightOdometer = new Odometer("intakeR", 60.79, true);
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

    public void driveStraight(double distance, double maxPower) {
        double minRampUp = .2;
        double minRampDown = .2;

        rawDrive(distance, distance, maxPower, minRampUp, minRampDown, .05, .00005);
    }

    public void pivot(double angle, double maxPower) {
        double minRampUp = .2;
        double minRampDown = .2;

        final double distancePer360 = 1001;
        double leftTarget = (angle * distancePer360)/360.0;
        double rightTarget = -1 * leftTarget;
        rawDrive(leftTarget, rightTarget, maxPower, minRampUp, minRampDown, .2, .05);
    }

    private void rawDrive(double leftTarget, double rightTarget, double maxPower, double minRampUp, double minRampDown, double upScale, double downScale){

        if(maxPower < 0 || maxPower > 1.0) {
            throw new Warning("maxPower " + maxPower + " must be between 0 and 1");
        }

        final double correctionScale = 0.005;

        leftOdometer.reset();
        rightOdometer.reset();

        try {
           while (true) {
                double leftDistance = -leftOdometer.getDistance();
                double rightDistance = rightOdometer.getDistance();

                double averageDistance = (Math.abs(leftDistance) + Math.abs(rightDistance)) / 2;

                double leftRemaining = Math.signum(leftTarget) * (leftTarget - leftDistance);
                double rightRemaining = Math.signum(rightTarget) * (rightTarget - rightDistance);

                double averageRemaining = (leftRemaining + rightRemaining) / 2;
                if (averageRemaining <= 0) {
                    break;
                }

                double rampUp, rampDown;
                double power;
                rampUp = Math.max(upScale * (averageDistance * Math.exp(2)), minRampUp);
                rampDown = Math.max(downScale * (averageRemaining * Math.exp(3)), minRampDown); //distanceTo accounts for flip across y axis and x offset

                power = Math.min(rampUp, Math.min(rampDown, maxPower));

                double adjustLeft = correctionScale * (averageDistance - Math.abs(leftDistance));
                double adjustRight = correctionScale * (averageDistance - Math.abs(rightDistance));

                power -= Math.max(adjustLeft, adjustRight);

                double powerLeft = Math.signum(leftTarget) * (power + adjustLeft);
                double powerRight = Math.signum(rightTarget) * (power + adjustRight);



                frontLeft.setPower(powerLeft);
                backLeft.setPower(powerLeft);
                frontRight.setPower(powerRight);
                backRight.setPower(powerRight);

               FTCUtilities.addData("power", power);
               FTCUtilities.addData("left r", leftRemaining);
               FTCUtilities.addData("right r", rightRemaining);
               FTCUtilities.addData("average remaining", averageRemaining);
               FTCUtilities.addData("leftDistance", leftDistance);
               FTCUtilities.addData("rightDistance", rightDistance);
               FTCUtilities.updateOpLogger();

           }
        } finally {
            setPowerAll(0);
        }
        Logger.getInstance().writeToFile();
    }

}
