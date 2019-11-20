package edu.ahs.robotics.hardware.sensors;

public class TriggerDistanceSenor extends DistanceSensor {
    private double threshold;

    public TriggerDistanceSenor(String sensorName, double threshold) {
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
