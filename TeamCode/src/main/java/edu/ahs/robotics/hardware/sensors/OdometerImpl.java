package edu.ahs.robotics.hardware.sensors;

import com.qualcomm.robotcore.hardware.DcMotor;

import edu.ahs.robotics.util.ftc.FTCUtilities;

/**
 * Non-Mock implementation of the Odometer interface that runs on the robot. Reads encoder values, tracks, and returns distance based on wheel diameter.
 *  <b>IMPORTANT:</b> setDirection() method on DcMotor changes encoder direction
 * @author Alex Appleby
 */
public class OdometerImpl implements Odometer {
    private DcMotor motor; //we use the crappy DcMotor class from FTC to access encoder values. Kind of a hack, but that's how it be.
    private double wheelCircumference;// in inches, used to be in mm
    private double ticksPerRotation; //specific to our S4T encoders. May be in need of change for other S4T models or different encoders.
    private int direction = 1; //enables canFlip. only 1 or -1. //*** IMPORTANT *** setDirection() method on DcMotor changes encoder direction

    /**
     * @param deviceName A string that ties the encoder sensor to the motor port of a DcMotor. Should be the same as whatever motor it's attached to.
     * @param wheelDiameter The diameter of the odometer wheel in inches. Likely needs to be tuned to reflect tolerances of wheel.
     * @param flip Whether or not this returns flipped values. Probably determine this by experimentation.
     */
    public OdometerImpl(String deviceName, double wheelDiameter, boolean flip, double ticksPerRotation) {
        motor = FTCUtilities.getMotor(deviceName);
        reset();
        wheelCircumference = wheelDiameter*Math.PI;
        if(flip){
            direction = -1;
        }
        this.ticksPerRotation = ticksPerRotation;
    }

    /**
     * @return Distance of rotation in inches.
     */
    @Override
    public double getDistance(){
        int ticks = motor.getCurrentPosition();
        double distance = (ticks*wheelCircumference) / ticksPerRotation; // rotations / ticks per rotation but combined for optimization
        return direction*distance;
    }

    /**
     * Resets encoder so that getDistance yields 0.
     */
    @Override
    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getRotations(){
        int ticks = motor.getCurrentPosition();
        double rotations = ticks/ ticksPerRotation;
        return direction*rotations;
    }
}
