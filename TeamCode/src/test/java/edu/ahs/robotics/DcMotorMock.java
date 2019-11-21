package edu.ahs.robotics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import edu.ahs.robotics.util.MotorHashService;


public class DcMotorMock implements DcMotor {

    private long startTime = System.currentTimeMillis();
    private double distance;
    private double motorPower;
    private RunMode runMode;
    private final double EFFICIENCY = 1.70
            ;
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
        encoderUpdate();
        return (int)distance;
    }

    @Override
    public void setMode(RunMode mode) {
        if(mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER){
            zeroTime();
            distance = 0;
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
        encoderUpdate();
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

    private void encoderUpdate(){//encoderUpdate allows for a more realistic modeling of motors by keeping a running tally of encoder value change rather than a static rate estimate
        double elapsedTime = System.currentTimeMillis() - startTime;
        zeroTime();
        double ticksPerTime = MotorHashService.getTicks(motorType) * MotorHashService.getRPMs(motorType)/60000 * motorPower * elapsedTime * EFFICIENCY;
        //ticks per rotation * rotations per millisecond * motorPower scalar * milliseconds * efficiency scalar
        distance += ticksPerTime;
    }

    private void zeroTime(){
        startTime = System.currentTimeMillis();
    }
}
