package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import edu.ahs.robotics.util.MotorHashService;


public class SingleDriveUnit extends DriveUnit{

    public SingleDriveUnit(String deviceName ,Config config, boolean flipped) {
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
        motor.setPower(Range.clip(motorPower, -1.0, 1.0));
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
