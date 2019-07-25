package edu.ahs.robotics;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.Map;

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

    public MecanumChassis(DriveUnit.Config driveUnitConfig, Map<ChassisMotors.Mecanum, Boolean> flips) {
        super();
        frontLeft = new SingleDriveUnit(FRONT_LEFT.getDeviceName(), driveUnitConfig, flips.get(FRONT_LEFT));
        frontRight = new SingleDriveUnit(FRONT_RIGHT.getDeviceName(), driveUnitConfig, flips.get(FRONT_RIGHT));
        backLeft = new SingleDriveUnit(BACK_LEFT.getDeviceName(), driveUnitConfig, flips.get(BACK_LEFT));
        backRight = new SingleDriveUnit(BACK_RIGHT.getDeviceName(), driveUnitConfig, flips.get(BACK_RIGHT));
    }

    public void execute(PlanElement planElement) {
        if (planElement instanceof ForwardMotion) {
            motionInterpreter((ForwardMotion) planElement);
        } else if (planElement instanceof ArcMotion) {
            motionInterpreter((ArcMotion) planElement);
        } else {
            throw new Warning("Couldn't find a way to execute planElement " + planElement.toString());
        }
    }

    private void motionInterpreter(ForwardMotion forwardMotion) {

        frontLeft.zeroDistance();
        frontRight.zeroDistance();
        backLeft.zeroDistance();
        backRight.zeroDistance();

        double encoderAverage = 0;


        while (encoderAverage < forwardMotion.travelDistance) {
            encoderAverage = (frontRight.getDistance() + frontLeft.getDistance() + backRight.getDistance() + backLeft.getDistance()) / 4;
            FTCUtilities.OpLogger("EncoderAverage", encoderAverage);
            setPowerAll(forwardMotion.motorPower);
        }
        setPowerAll(0);
    }

    private void motionInterpreter(ArcMotion arcMotion) {

    }

    private void setPowerAll(double motorPower) {
        frontRight.setPower(motorPower);
        frontLeft.setPower(motorPower);
        backRight.setPower(motorPower);
        backLeft.setPower(motorPower);
    }
}
