package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import edu.ahs.robotics.util.ftc.FTCUtilities;
import edu.ahs.robotics.util.MotorHashService;

public abstract class DriveUnit {

    protected boolean flipped;
    protected Config config;
    protected double wheelCircumference;
    protected String deviceName;
    protected DcMotor motor;



    public DriveUnit(String deviceName, Config config, boolean flipped) {
        motor = FTCUtilities.getMotor(deviceName);
        wheelCircumference = config.wheelDiameter * Math.PI;
        this.config = config;
        this.flipped = flipped;

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public abstract void setPower(double power);
    public abstract void zeroDistance();
    public abstract double getDistance();

    public static class Config {
        private GearRatio gearRatio;
        private double wheelDiameter;
        private MotorHashService.MotorTypes motorType;

        public Config(GearRatio gearRatio, double wheelDiameter, MotorHashService.MotorTypes motorType) {
            this.gearRatio = gearRatio;
            this.wheelDiameter = wheelDiameter;
            this.motorType = motorType;
        }

        public GearRatio getGearRatio() {
            return gearRatio;
        }

        public double getWheelDiameter() {
            return wheelDiameter;
        }

        public MotorHashService.MotorTypes getMotorType() {
            return motorType;
        }
    }

}


