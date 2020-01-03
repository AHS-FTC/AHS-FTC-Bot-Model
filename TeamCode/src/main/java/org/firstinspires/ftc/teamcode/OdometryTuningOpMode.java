package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import edu.ahs.robotics.hardware.sensors.OdometerImpl;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.Logger;
//1738 mm
//left = 9.175 rots = 60.2967 mm
//right = 9.1 rots = 60.79
@TeleOp(name="Odometry Tuner", group="Iterative Opmode")
//@Disabled
public class OdometryTuningOpMode extends OpMode {
    private DcMotor FL, FR, BL, BR;
    private OdometerImpl l, r;
    private double leftDiameter = 60, rightDiameter = 60;

    @Override
    public void init() {
        FTCUtilities.setOpMode(this);

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    public void init_loop(){
        leftDiameter += gamepad1.left_stick_y*0.001;
        rightDiameter += gamepad1.right_stick_y*0.001;

        telemetry.addData("left Diameter", leftDiameter);
        telemetry.addData("right Diameter", rightDiameter);
        telemetry.update();
    }

    public void start(){
        l = new OdometerImpl("intakeL", leftDiameter, false,1440);
        r = new OdometerImpl("intakeR", rightDiameter, true,1440);

        l.reset();
        r.reset();

        //Logger.append(Logger.Cats.DESIDIST, String.valueOf(leftDiameter));
        //Logger.append(Logger.Cats.DDADJUSTMENT, String.valueOf(rightDiameter));
    }

    public void loop(){
        //Logger.append(Logger.Cats.MOTORPOW, String.valueOf(l.getRotations()));
        //Logger.append(Logger.Cats.ENCODERDIST, String.valueOf(r.getRotations()));

        telemetry.addData("left rots", l.getRotations());
        telemetry.addData("right rots", r.getRotations());
        telemetry.update();
    }
    public void stop(){
        //Logger.getInstance().stopWriting();
    }
}
