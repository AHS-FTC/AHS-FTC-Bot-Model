package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import edu.ahs.robotics.hardware.sensors.LimitSwitch;
import edu.ahs.robotics.util.FTCUtilities;

public class Slides {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private LimitSwitch limitSwitch;
    private double motorPower;
    private int encoderTicksPerLevel = 420;

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

    private void resetEncoder(DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setEncoderModeRunToPostion(DcMotor motor){
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runSlidesToEncoder(double encoderHeight) {
        while(leftMotor.getCurrentPosition() < encoderHeight) {
            leftMotor.setPower(motorPower);
            rightMotor.setPower(-motorPower);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

    public void runSlidesToLevel(int level) {
        if(level < 1 || level > 10) {
            throw new Warning("level "+ level +" must be between 1 and 10");
        }
        int level1 = 200;
        int ticksAtLevel;
        ticksAtLevel = (level-1) * encoderTicksPerLevel + level1;
        leftMotor.setTargetPosition(ticksAtLevel);
        rightMotor.setTargetPosition(ticksAtLevel);
        setEncoderModeRunToPostion(leftMotor);
        setEncoderModeRunToPostion(rightMotor);
        leftMotor.setPower(motorPower); //These are powers for going up
        rightMotor.setPower(motorPower);

    }

    public void resetSlidesToOriginalPosition() {
        leftMotor.setPower(-motorPower); //These are powers for going down
        rightMotor.setPower(motorPower);
        while(!limitSwitch.isTriggered()) {}
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        resetEncoders();

    }

    public void resetEncoders() {
        resetEncoder(leftMotor);
        resetEncoder(rightMotor);
    }

}
