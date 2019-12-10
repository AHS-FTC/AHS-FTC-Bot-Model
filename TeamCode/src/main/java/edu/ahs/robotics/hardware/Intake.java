package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import edu.ahs.robotics.hardware.sensors.Trigger;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.util.FTCUtilities;

public class Intake { //todo make a one or two motor alternate to intake class
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private IntakeMode intakeMode;
    private double motorPower;

    public Intake(double motorPower) {
        leftMotor = FTCUtilities.getMotor("intakeL");
        rightMotor = FTCUtilities.getMotor("intakeR");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.motorPower = motorPower;
        intakeMode = IntakeMode.OFF;
    }

    public enum IntakeMode {
        OFF,
        IN,
        OUT
    }

    /**
     * Runs intake until triggered, monitors on a new thread. Nonblocking method
     * @param trigger the trigger which stops the intake when triggered
     */
    public void startIntakeWaitForBlock(TriggerDistanceSensor trigger) {
        intakeMode = IntakeMode.IN;
        runMotorsByMode();
        BlockMonitor blockMonitor = new BlockMonitor(trigger);
        Thread thread = new Thread(blockMonitor);
        thread.start();
    }

    private void runMotorsByMode() {
        if (intakeMode == IntakeMode.IN) {
            runMotors(motorPower);
        } else if (intakeMode == IntakeMode.OUT) {
            runMotors(-motorPower);
        } else if (intakeMode == IntakeMode.OFF) {
            stopMotors();
        }
    }

    public void runMotors(double motorPower) {
        leftMotor.setPower(motorPower);
        rightMotor.setPower(motorPower);
    }

    public void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private class BlockMonitor implements Runnable {
        private Trigger stopTrigger;

        public BlockMonitor(Trigger stopTrigger) {
            this.stopTrigger = stopTrigger;
        }

        @Override
        public void run() {
            while (!stopTrigger.isTriggered()) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {

                }
            }
            stopMotors();
        }

    }
}
