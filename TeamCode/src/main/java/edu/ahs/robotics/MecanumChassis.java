package edu.ahs.robotics;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.HashMap;

public class MecanumChassis extends Chassis {

    private SingleDriveUnit frontLeft;
    private SingleDriveUnit frontRight;
    private SingleDriveUnit backLeft;
    private SingleDriveUnit backRight;

    // Device names for drive units; corresponds to
    private static String FRONT_LEFT = "FL";
    private static String FRONT_RIGHT = "FR";
    private static String BACK_LEFT = "BL";
    private static String BACK_RIGHT = "BR";

    public MecanumChassis(DriveUnit.Config driveUnitConfig) {
        super();
        frontLeft = new SingleDriveUnit(FRONT_LEFT, driveUnitConfig, false);
        frontRight = new SingleDriveUnit(FRONT_RIGHT, driveUnitConfig, true);
        backLeft = new SingleDriveUnit(BACK_LEFT, driveUnitConfig, false);
        backRight = new SingleDriveUnit(BACK_RIGHT, driveUnitConfig, true);
    }

    public void execute(PlanElement planElement) {
        if (planElement instanceof ForwardMotion) {
            motionInterpreter((ForwardMotion) planElement);
        } else if (planElement instanceof ArcMotion) {
            motionInterpreter((ArcMotion) planElement);
        } else {
            throw new Warning("Couldn't find a way to execute Planelement " + planElement.toString());
        }
    }

    private void motionInterpreter(ForwardMotion forwardMotion) {

        //frontLeft.zeroDistance();
        //frontRight.zeroDistance();
        //backLeft.zeroDistance();
        //backRight.zeroDistance();

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
