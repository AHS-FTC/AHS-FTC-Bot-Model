package edu.ahs.robotics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


public class DcMotorMock implements DcMotor {

    private long startTime = System.currentTimeMillis();
    private double motorPower;
    private RunMode runMode;
    private final double EFFICIENCY = .2;
    private MotorHashService.MotorTypes motorType;
    private Direction direction = Direction.FORWARD;

    @Override
    public MotorConfigurationType getMotorType() {
        return null;
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {

    }

    @Override
    public DcMotorController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return null;
    }

    @Override
    public void setPowerFloat() {

    }

    @Override
    public boolean getPowerFloat() {
        return false;
    }

    @Override
    public void setTargetPosition(int position) {

    }

    @Override
    public int getTargetPosition() {
        return 0;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public int getCurrentPosition() {
        long elapsedTime = System.currentTimeMillis() - startTime;
        double ticksPerTime = MotorHashService.getTicks(motorType) * MotorHashService.getRPMs(motorType) * motorPower * (elapsedTime/1000.0) * EFFICIENCY;
        return (int) (ticksPerTime);
    }

    @Override
    public void setMode(RunMode mode) {
        if(mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER){
            startTime = System.currentTimeMillis();
            runMode = mode;
        } if (mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            runMode = mode;
        }
        else {
            throw new UnsupportedOperationException("The RunMode you tried to use has not been set up with class DcMotorMock");
        }

    }

    @Override
    public RunMode getMode() {
        return runMode;
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
        //throw new UnsupportedOperationException("SetDirection has yet to be implemented in class DcMotorMock");
    }

    @Override
    public Direction getDirection() {
        return direction;
        //throw new UnsupportedOperationException("GetDirection has yet to be implemented in class DcMotorMock");
    }

    @Override
    public void setPower(double power) {
        motorPower = power;

    }

    @Override
    public double getPower() {
        return 0;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    DcMotorMock(MotorHashService.MotorTypes motorType) {
        this.motorType = motorType;
    }

    @Override
    public void close() {

    }
}
