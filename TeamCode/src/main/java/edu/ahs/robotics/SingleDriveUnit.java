package edu.ahs.robotics;

import com.qualcomm.robotcore.hardware.DcMotor;


public class SingleDriveUnit extends DriveUnit{
    int flip = 1;


    public SingleDriveUnit(String deviceName,Config config, boolean flipped) {
        super(deviceName, config, flipped);
        this.deviceName = deviceName;
        this.config = config;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FTCUtilities.OpLogger("flop",flip);
        if (flipped){
            flip = -1;
        }

    }

    public void setPower(double motorPower){
        if (Math.abs(motorPower) > 1) {
            throw new Error("DriveUnit motorPower is not between 1 and -1");
        }
        motor.setPower(flip*motorPower);
    }

    public void zeroDistance(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getDistance (){
        double ticksPerRotation = MotorHashService.getTicks(config.getMotorType());
        double rotations = motor.getCurrentPosition()/ticksPerRotation;
        double rotationsAfterGears = rotations*config.getGearRatio().getRatioAsDouble();
        double inchesTraveled = wheelCircumference * rotationsAfterGears;
        return flip*inchesTraveled;
    }
}
