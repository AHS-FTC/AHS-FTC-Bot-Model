package edu.ahs.robotics.hardware.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Wrapper class to manage the FTC BNO055IMU class and facilitate mocking in the future
 * @author Alex Appleby
 */
public class IMU {
    private BNO055IMU imu;
    double correction;
    private Orientation lastAngles = new Orientation();
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    double lastAngle;
    double offset = 0;

    public IMU(BNO055IMU imu) {
        this.imu = imu;

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);
    }

    /**
     * Gets the IMU Heading. Oversteps the internal IMU behavior that returns degrees between 180 and -180, meaning this method can return beyond 360 and -360
     * @return Heading in degrees
     */
    public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle-lastAngle;
        lastAngle = angles.firstAngle;

        if (deltaAngle < -180){
            offset+=360;
        } else if (deltaAngle>180){
            offset -=360;
        }

        return angles.firstAngle + offset;
    }

    public boolean isCalibrated (){
        return imu.isGyroCalibrated();
    }
    /*
    public double getGlobalHeading(){
        update();
        return lastAngles.firstAngle;
    }
    */

}