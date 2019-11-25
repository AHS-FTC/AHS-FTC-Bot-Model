package edu.ahs.robotics.autocommands.obmcommands;

import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.hardware.sensors.Trigger;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.util.FTCUtilities;

public class IntakeMonitor implements Runnable {
    Trigger stopTrigger;
    Intake intake;

    public IntakeMonitor(Trigger stopTrigger, Intake intake) {
        this.stopTrigger = stopTrigger;
        this.intake = intake;
    }

    public void start(){

    }

    @Override
    public void run() {
        while (!stopTrigger.isTriggered()){
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {

            }
        }
        intake.stopMotors();
    }
}
