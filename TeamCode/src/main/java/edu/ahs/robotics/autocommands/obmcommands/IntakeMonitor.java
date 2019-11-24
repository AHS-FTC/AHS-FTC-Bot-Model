package edu.ahs.robotics.autocommands.obmcommands;

import edu.ahs.robotics.hardware.Intake;
import edu.ahs.robotics.hardware.sensors.TriggerDistanceSensor;
import edu.ahs.robotics.util.FTCUtilities;

public class IntakeMonitor implements Runnable {
    TriggerDistanceSensor stopTrigger;
    Intake intake;

    public IntakeMonitor(TriggerDistanceSensor stopTrigger, Intake intake) {
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
