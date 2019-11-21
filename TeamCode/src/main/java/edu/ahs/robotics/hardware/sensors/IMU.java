package edu.ahs.robotics.hardware.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import edu.ahs.robotics.util.FTCUtilities;

public class IMU {
    private BNO055IMU imu;
    double correction;
    private Orientation lastAngles = new Orientation();
    double globalAngle;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public IMU(BNO055IMU imu) {
        this.imu = imu;

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);
    }

    public double getHeading(){
        update();
        return lastAngles.firstAngle;
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
    public double getCorrection(){
        update();
        return correction;
    }
    private void update(){
        correction = checkDirection();
    }

    private double checkDirection(){
        double correction, angle, gain = 0.15;
        angle = getAngle();
        if (angle == 0){
            correction = 0;
        } else{
            correction = -angle;
        }
        correction = correction*gain;

        return correction;
    }
    private double getAngle(){
        FTCUtilities.OpLogger("imu", imu);
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180){
            deltaAngle+=360;
        } else if (deltaAngle>180){
            deltaAngle -=360;
        }
        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}