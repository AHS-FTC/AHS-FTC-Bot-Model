package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import edu.ahs.robotics.hardware.sensors.LimitSwitch;
import edu.ahs.robotics.util.FTCUtilities;

public class Slides {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private double motorPower;
    private LimitSwitch limitSwitch;

    public Slides (double motorPower, LimitSwitch limitSwitch){
        leftMotor = FTCUtilities.getMotor("slideL");
        rightMotor = FTCUtilities.getMotor("slideR");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.motorPower = motorPower;
        this.limitSwitch = limitSwitch;
    }

    public void runSlidesToEncoder(double encoderHeight) {
        while(leftMotor.getCurrentPosition() < encoderHeight) {
            leftMotor.setPower(motorPower);
            rightMotor.setPower(motorPower);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

    public void resetSlides() {
        while(!limitSwitch.isTriggered()) {
            leftMotor.setPower(-motorPower);
            rightMotor.setPower(-motorPower);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

}
