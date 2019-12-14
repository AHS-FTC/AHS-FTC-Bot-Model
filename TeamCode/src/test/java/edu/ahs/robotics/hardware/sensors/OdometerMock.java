package edu.ahs.robotics.hardware.sensors;

import edu.ahs.robotics.hardware.sensors.Odometer;

public class OdometerMock implements Odometer {

    private double[] inputs;
    private int index = 0;

    public OdometerMock(double[] inputs) {
        this.inputs = inputs;
    }

    @Override
    public double getDistance() {
        if(index <= inputs.length - 1) {//todo check +1 -1
            index ++;
            return inputs[index-1]; // -1 to compensate for ++
        } else {
            return inputs[inputs.length - 1];//todo check +1 -1
        }
    }

    @Override
    public void reset() {

    }
}
