package edu.ahs.robotics;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DoubleDriveUnit extends DriveUnit{
    private GearRatio gearRatio;
    private double wheelDiameter; // in inches
    private double wheelCircumference;
    DcMotor motor1;
    DcMotor motor2;
    String deviceName;
    MotorHashService.MotorTypes motorType;
    int flip = 1;

    public DoubleDriveUnit(String deviceName, BotFactory botFactory, boolean isFlipped) {
        this.gearRatio = botFactory.getDriveGearRatio();  // input over output gear ratio
        this.wheelDiameter = botFactory.getWheelDiameter();
        wheelCircumference = wheelDiameter * Math.PI;
        this.deviceName = deviceName;
        this.motorType = botFactory.getDriveMotorType();

        motor1 = FTCUtilities.getMotor(deviceName);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor2 = FTCUtilities.getMotor(deviceName);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FTCUtilities.OpLogger("flop",flip);
        if (isFlipped){
            flip = -1;
        }

//        if(isFlipped){
//            motor.setDirection(DcMotorSimple.Direction.REVERSE);
//        } else{
//            motor.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
    }
    public void setPower(double motorPower){
        if (Math.abs(motorPower) > 1) {
            throw new Error("SingleDriveUnit motorPower is not between 1 and -1");
        }
        motor1.setPower(flip*motorPower);
        motor2.setPower(flip*motorPower);
    }

    public void zeroDistance(){
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getDistance (){
        double ticksPerRotation = MotorHashService.getTicks(motorType);
        double rotations = motor1.getCurrentPosition()/ticksPerRotation;
        rotations+= motor2.getCurrentPosition()/ticksPerRotation;
        double rotationsAfterGears = (rotations/2)*gearRatio.getRatioAsDouble();
        double inchesTraveled = wheelCircumference * rotationsAfterGears;
        return flip*inchesTraveled;
    }
}
