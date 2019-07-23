package edu.ahs.robotics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class SingleDriveUnit extends DriveUnit{
    private GearRatio gearRatio;
    private double wheelDiameter; // in inches
    private double wheelCircumference;
    DcMotor motor;
    String deviceName;
    MotorHashService.MotorTypes motorType;
    int flip = 1;


    public SingleDriveUnit(String deviceName, BotConfig botConfig, boolean isFlipped) {
        this.gearRatio = botConfig.getDriveGearRatio();  // input over output gear ratio
        this.wheelDiameter = botConfig.getWheelDiameter();
        wheelCircumference = wheelDiameter * Math.PI;
        this.deviceName = deviceName;
        this.motorType = botConfig.getDriveMotorType();

        motor = FTCUtilities.getMotor(deviceName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FTCUtilities.OpLogger("flop",flip);
        if (isFlipped){
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else{motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

//        if(isFlipped){
//            motor.setDirection(DcMotorSimple.Direction.REVERSE);
//        } else{
//            motor.setDirection(DcMotorSimple.Direction.FORWARD);
//        }


    }


    public void setPower(double motorPower){
        FTCUtilities.OpLogger("Motor Power", motorPower);
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
        double ticksPerRotation = MotorHashService.getTicks(motorType);
        double rotations = motor.getCurrentPosition()/ticksPerRotation;
        double rotationsAfterGears = rotations*gearRatio.getRatioAsDouble();
        double inchesTraveled = wheelCircumference * rotationsAfterGears;
        return inchesTraveled;
    }
}
