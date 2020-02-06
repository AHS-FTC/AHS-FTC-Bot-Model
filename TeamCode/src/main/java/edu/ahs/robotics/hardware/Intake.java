package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import edu.ahs.robotics.hardware.sensors.Trigger;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.util.FTCUtilities;

public class Intake { //todo make a one or two motor alternate to intake class
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private double motorPower;
    //private BlockMonitor blockMonitor;

    public Intake(double motorPower) {
        leftMotor = FTCUtilities.getMotor("intakeL");
        rightMotor = FTCUtilities.getMotor("intakeR");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.motorPower = motorPower;

        //blockMonitor = new BlockMonitor();
        //blockMonitor.start();
    }

    /**
     * Runs intake until triggered, monitors on a new thread. Nonblocking method
     * @param trigger the trigger which stops the intake when triggered
     */
    public void startIntakeWaitForBlock(TriggerDistanceSensor trigger) {
        runMotors(motorPower);
        //blockMonitor.setStopTrigger(trigger);
        FTCUtilities.addData("is triggered", trigger.isTriggered());
        FTCUtilities.updateOpLogger();
        FTCUtilities.sleep(1000);
        //blockMonitor.makeActive();
    }

    public void runMotors(double motorPower) {
        leftMotor.setPower(motorPower);
        rightMotor.setPower(motorPower);
    }

    public void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        //blockMonitor.pause();
    }

    private class BlockMonitor extends Thread {
        private Trigger stopTrigger;
        private volatile boolean running = true;
        private volatile boolean active = false;

        public BlockMonitor() {
        }

        public void setStopTrigger(Trigger stopTrigger){
            this.stopTrigger = stopTrigger;
        }

        public synchronized void makeActive(){
            active = true;
            notifyAll();
        }

        public void pause(){
            active = false;
        }

        public synchronized void kill(){
            running = false;
            notifyAll();
        }
        @Override
        public void run() {
            while (running) {
                if(!active){
                    synchronized (this) {
                        try {
                            wait();
                        } catch (InterruptedException e) {

                        }
                    }
                }
                if(!running){ //break if running is flipped during sleep
                    break;
                }
                if (stopTrigger.isTriggered()) {
                    stopMotors();
                } else {
                    synchronized (this) {
                        try {
                            wait(50);
                        } catch (InterruptedException e) {

                        }
                    }
                }
            }
        }
    }
}
