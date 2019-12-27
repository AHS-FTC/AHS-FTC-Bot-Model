package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;

import edu.ahs.robotics.util.MotorHashService;

/**
 * DcMotorMock that sneakily stores motor positions to facilitate in testing.
 * @author Alex Appleby
 */
public class DcMotorMockLogger implements DcMotor {

    ArrayList<Double> powerList;

    private long startTime = System.currentTimeMillis();
    private double motorPower;
    private RunMode runMode;
    private MotorHashService.MotorTypes motorType;
    private Direction direction = Direction.FORWARD;

    public DcMotorMockLogger() {
        powerList = new ArrayList<>();
    }

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
        return 0;
    }

    @Override
    public void setMode(RunMode mode) {
        runMode = mode;
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
        powerList.add(power);
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

    @Override
    public void close() {

    }

    public ArrayList<Double> getPowerList(){
        return powerList;
    }
}
