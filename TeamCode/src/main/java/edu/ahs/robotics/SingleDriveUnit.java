package edu.ahs.robotics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


public class SingleDriveUnit extends DriveUnit{
    int flip = 1;


    public SingleDriveUnit(String deviceName,Config config, boolean flipped) {
        super(deviceName, config, flipped);
        this.deviceName = deviceName;
        this.config = config;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        if (flipped){
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else{motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

    }

    public void setPower(double motorPower){
        motor.setPower(Range.clip(motorPower,-1,1));
    }

    public void adjustPower(double powerAdjustment){
        setPower(motor.getPower()+powerAdjustment);
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
        return inchesTraveled;
    }
}
