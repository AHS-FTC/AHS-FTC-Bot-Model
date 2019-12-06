package edu.ahs.robotics.hardware.sensors;

import edu.ahs.robotics.util.FTCUtilities;

public class TriggerDistanceSensor extends DistanceSensor implements Trigger {
    private double threshold;
    private long delay;

    public TriggerDistanceSensor(String sensorName, double threshold, long delay) {
        super(sensorName);
        this.threshold = threshold;
        this.delay = delay;
    }

    public boolean isTriggered(){
        if (getDist() <= threshold){
            FTCUtilities.sleep(delay);
            return true;
        } else {
            return false;
        }
    }
}


