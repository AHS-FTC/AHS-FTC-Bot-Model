package edu.ahs.robotics.hardware.sensors;

public class TriggerDistanceSensor extends DistanceSensor implements Trigger {
    private double threshold;

    public TriggerDistanceSensor(String sensorName, double threshold) {
        super(sensorName);
        this.threshold = threshold;
    }

    public boolean isTriggered(){
        if (getDist() <= threshold){
            return true;
        } else {
            return false;
        }
    }
}
