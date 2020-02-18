package org.firstinspires.ftc.teamcode.pathtests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ahs.robotics.hardware.sensors.DistanceSensor;
import edu.ahs.robotics.util.FTCMath;
import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.RingBuffer;
import edu.ahs.robotics.util.opmodes.LinearOpMode16896;

@TeleOp(name = "Distance Sensor thingee", group = "Linear Opmode")
//@Disabled
public class DistanceSensorOpMode extends LinearOpMode16896 {

    @Override
    protected void runProgram() {
        DistanceSensor leftDistanceSensor = new DistanceSensor("leftDistance");
        DistanceSensor rightDistanceSensor = new DistanceSensor("rightDistance");

        RingBuffer<Double> leftDistanceSensorReadings = new RingBuffer<>(30,0.0);
        RingBuffer<Double> rightDistanceSensorReadings = new RingBuffer<>(30,0.0);

        waitForStart();

        while (opModeIsActive()){
            long lastTime = FTCUtilities.getCurrentTimeMillis();

            double leftReading = leftDistanceSensor.getDist();
            double rightReading = rightDistanceSensor.getDist();

            telemetry.addData("left reading", leftReading);
            telemetry.addData("right reading", rightReading);

            leftDistanceSensorReadings.insert(leftReading);

            rightDistanceSensorReadings.insert(rightReading);

            telemetry.addData("leftStandardDev", FTCMath.ringBufferStandardDeviation(leftDistanceSensorReadings));
            telemetry.addData("rightStandardDev", FTCMath.ringBufferStandardDeviation(rightDistanceSensorReadings));

            if(leftReading > rightReading){
                telemetry.addLine("go left");
            } else {
                telemetry.addLine("go right");
            }

            telemetry.update();
        }

    }
}
