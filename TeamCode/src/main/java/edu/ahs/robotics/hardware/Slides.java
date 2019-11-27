package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import edu.ahs.robotics.hardware.sensors.LimitSwitch;
import edu.ahs.robotics.util.FTCUtilities;

public class Slides {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private LimitSwitch limitSwitch;
    private double motorPower;
    private static final int ENCODER_TICKS_PER_LEVEL = 420;
    private static final int SLIDES_MAX = 4150;

    public Slides (double motorPower){
        leftMotor = FTCUtilities.getMotor("slideL");
        rightMotor = FTCUtilities.getMotor("slideR");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limitSwitch = new LimitSwitch("limitSwitch");

        this.motorPower = motorPower;
    }

    public void runSlides(double slidesPower) {
        if (getCurrentPosition() >= SLIDES_MAX) {
            slidesPower = Range.clip(slidesPower, -1, 0);
        } else if (limitSwitch.isTriggered()) {
            slidesPower = Range.clip(slidesPower, 0, 1);
            resetEncoders();
        }

        setPower(slidesPower);

    }

    public boolean atBottom(){
        return limitSwitch.isTriggered();
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
        ticksAtLevel = (level-1) * ENCODER_TICKS_PER_LEVEL + level1;
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

    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void stopMotors() {
        setPower(0);
    }

    public double getCurrentPosition() {
        double slidesPosition = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition())/2;
        return slidesPosition;
    }

}
