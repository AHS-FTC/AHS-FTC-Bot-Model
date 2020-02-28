package edu.ahs.robotics.util.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.ahs.robotics.hardware.sensors.DistanceSensor;
import edu.ahs.robotics.util.ftc.FTCMath;
import edu.ahs.robotics.util.ftc.FTCUtilities;
import edu.ahs.robotics.util.ftc.RingBuffer;
import edu.ahs.robotics.util.opmodes.bfr.LinearOpMode16896;

@TeleOp(name = "Distance Sensor thingee", group = "Linear Opmode")
@Disabled
public class DistanceSensorOpMode extends LinearOpMode16896 {

    @Override
    protected void runProgram() {
        DistanceSensor leftDistanceSensor = new DistanceSensor("leftDistance");
        DistanceSensor rightDistanceSensor = new DistanceSensor("rightDistance");

        RingBuffer<Double> leftDistanceSensorReadings = new RingBuffer<>(30,0.0);
        RingBuffer<Double> rightDistanceSensorReadings = new RingBuffer<>(30,0.0);

        waitForStart();

        long lastTime = FTCUtilities.getCurrentTimeMillis();

        while (opModeIsActive()){
            double leftReading = leftDistanceSensor.getDist();
            double rightReading = rightDistanceSensor.getDist();

            telemetry.addData("left reading", leftReading);
            telemetry.addData("right reading", rightReading);

            leftDistanceSensorReadings.insert(leftReading);

            rightDistanceSensorReadings.insert(rightReading);

            telemetry.addData("leftStandardDev", FTCMath.ringBufferStandardDeviation(leftDistanceSensorReadings));
            telemetry.addData("rightStandardDev", FTCMath.ringBufferStandardDeviation(rightDistanceSensorReadings));

            telemetry.addData("deltaTime", FTCUtilities.getCurrentTimeMillis() - lastTime);

            if(leftReading > rightReading){
                telemetry.addLine("go left");
            } else {
                telemetry.addLine("go right");
            }

            telemetry.update();

            lastTime = FTCUtilities.getCurrentTimeMillis();
        }

    }
}
