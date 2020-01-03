package edu.ahs.robotics.util.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import edu.ahs.robotics.hardware.sensors.IMU;
import edu.ahs.robotics.hardware.sensors.Odometer;
import edu.ahs.robotics.hardware.sensors.OdometerImpl;
import edu.ahs.robotics.seasonrobots.Ardennes;
import edu.ahs.robotics.util.FTCUtilities;


/**
 * Automatically calculates odometer constants.
 * </br>
 * Based on 9794 Wizards.exe calibration OpMode but theirs was shitty.
 * @author Alex Appleby
 */
@TeleOp(name = "Odometry System Calibration", group = "Calibration")
//@Disabled
public class OdometryCalibration extends OpMode {

    Odometer left, right, back;

    Ardennes ardennes;


    private static final double ROTATION_POWER = 0.5; //tune this. maybe. make robot turn even.

    private double distanceFromCenter1;
    private double distanceFromCenter2;
    private double backWheelInchesPerDegree;

    @Override
    public void init() {
        FTCUtilities.setOpMode(this);

        left = new OdometerImpl("intakeL", 2.3596,false,1440); // tune these. Make sure odometers rotate forward
        right = new OdometerImpl("intakeR", 2.3617,true,1440); // tune these
        back = new OdometerImpl("BR", 2.387,false,4000); // tune these

        left.reset();
        right.reset();
        back.reset();

        ardennes = new Ardennes();
    }

    @Override
    public void init_loop(){
        updateTelemetry();
    }

    @Override
    public void start() {
        double deltaDegrees = 360;

        ardennes.getChassis().pivot(360,.7);

        distanceFromCenter1 = left.getDistance()/Math.PI;
        distanceFromCenter2 = right.getDistance()/Math.PI;

        backWheelInchesPerDegree = back.getDistance() / deltaDegrees;
    }

    @Override
    public void loop() {

        telemetry.addData("distance from center 1", distanceFromCenter1);
        telemetry.addData("distance from center 2", distanceFromCenter2);
        telemetry.addData("back wheel inches per degree", backWheelInchesPerDegree);
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("left - ins", left.getDistance());
        telemetry.addData("right - ins", right.getDistance());
        telemetry.addData("back - ins", back.getDistance());
        telemetry.update();
    }

}
