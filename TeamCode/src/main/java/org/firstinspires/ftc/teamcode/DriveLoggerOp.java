/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.ahs.robotics.util.Logger;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DriveLogger", group="Iterative Opmode")
@Disabled
public class DriveLoggerOp extends LinearOpMode {

    private final double Y_DIST_FROM_CENTER = 6.47722 -
            0.5703549745; // 6.25 measured,  otherwise calculated expiramentally via desmos. nice!
    private ElapsedTime runtime = new ElapsedTime();
    double lastHeading = 0;
    double offset = 0;

    @Override
    public void runOpMode() {

        BNO055IMU imu;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        //parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        DcMotor frontLeft = hardwareMap.get(DcMotor.class,"FL");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "FR");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "BL");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor x = hardwareMap.get(DcMotor.class,"x");
        DcMotor y = hardwareMap.get(DcMotor.class, "y");


        x.setDirection(DcMotorSimple.Direction.FORWARD);
        y.setDirection(DcMotorSimple.Direction.FORWARD);


        int xTicks, yTicks;
        int xTicksLast = 0, yTicksLast = 0;
        int dxTicks, dyTicks;
        double dxRots, dyRots;
        double dxIns, dyIns;
        double heading, dHeading, headingLast = 0;
        double yPrediction;

        double xRadius, yRadius;
        double dxLocal, dyLocal; //change in local coords
        double localX = 0, localY = 0;
        double dxGlobal, dyGlobal;//todo

        double globalX = 0, globalY = 0;

        String coords;



        waitForStart();
        runtime.startTime();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            //record the z axis

            heading = getAngle(imu);

            dHeading = heading - headingLast;

            headingLast = heading;

            telemetry.addData("IMU degs", Math.toDegrees(heading));
            //telemetry.addData("IMU rads", targetHeading);

            xTicks = x.getCurrentPosition();
            yTicks = y.getCurrentPosition();

            dxTicks = xTicks - xTicksLast;
            dyTicks = yTicks - yTicksLast;

            xTicksLast = xTicks;
            yTicksLast = yTicks;

            dxRots = (double)dxTicks/1440.0;//360 cpr times 4
            dyRots = (double)dyTicks/1440.0;

            //Factor out the dY attributed to rotation
            yPrediction = dHeading*Y_DIST_FROM_CENTER; //Todo fix

            dxIns = dxRots*(2.3622*Math.PI);
            dyIns = dyRots*(2.3622*Math.PI) - yPrediction;

            //arc length = theta*r
            if(false) {
                xRadius = dxIns/dHeading;
                yRadius = dyIns/dHeading;

                dyLocal = yRadius*(1-Math.cos(dHeading) + xRadius*(Math.sin(dHeading)));
                dxLocal = yRadius*(Math.sin(dHeading)) - xRadius*(1-Math.cos(dHeading));
            }else{
                dxLocal = dxIns;
                dyLocal = dyIns;
            }



            globalX += dxLocal*Math.sin(heading) + dyLocal*Math.cos(heading);
            globalY += dxLocal*Math.cos(heading) + dyLocal*Math.sin(heading);

            localX += dxLocal;
            localY += dyLocal;

            //telemetry.addData("x", xTicks);
            //telemetry.addData("y", yTicks);

            telemetry.addData("dx", dxTicks);
            telemetry.addData("dy", dyTicks);
            telemetry.addData("dHeading", dHeading);



            coords = globalX + ", " + globalY + " , " + heading;

            telemetry.addData("local X", localX);
            telemetry.addData("local Y", localY);
            telemetry.addData("time", runtime.milliseconds());


            //Logger.append(Logger.Cats.TIME,String.valueOf(runtime.milliseconds()));
            //Logger.append(Logger.Cats.DADJUSTMENT, String.valueOf(Math.toDegrees(heading)));
            //Logger.append(Logger.Cats.POWADJ, String.valueOf(dyIns/dHeading));
            //Logger.append(Logger.Cats.POWADJ, String.valueOf(yTicks));
            //Logger.append(Logger.Cats.MOTORPOW,String.valueOf(dHeading));
            //Logger.append(Logger.Cats.MOTORPOW,);

            //Logger.append(Logger.Cats.DESIDIST, String.valueOf(dyTicks));
            //Logger.append(Logger.Cats.MOTORPOW, String.valueOf(localY));
            //Logger.append(Logger.Cats.ERROR, String.valueOf(dyIns));

            frontLeft.setPower(gamepad1.right_stick_y+gamepad1.right_stick_x+gamepad1.left_stick_x); //todo flip strafe and rot
            frontRight.setPower(gamepad1.right_stick_y-gamepad1.right_stick_x-gamepad1.left_stick_x);
            backLeft.setPower(gamepad1.right_stick_y-gamepad1.right_stick_x+gamepad1.left_stick_x);
            backRight.setPower(gamepad1.right_stick_y+gamepad1.right_stick_x-gamepad1.left_stick_x);

            telemetry.update();
        }
        //Logger.getInstance().stopWriting();
    }

    private double getAngle(BNO055IMU imu) {
        double heading = imu.getAngularOrientation().firstAngle;
        double deltaHeading = heading - lastHeading;

        if(deltaHeading < -Math.PI){
            offset += 2*Math.PI;
        } else if(deltaHeading > Math.PI){
            offset -= 2*Math.PI;
        }

        lastHeading = heading;

        return heading + offset;
    }
}
