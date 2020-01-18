package edu.ahs.robotics.hardware.sensors;

import com.qualcomm.robotcore.hardware.TouchSensor;

import edu.ahs.robotics.util.FTCUtilities;

public class LimitSwitch implements Trigger {
    TouchSensor limitSwitch;
    boolean flipped;

    public LimitSwitch(String deviceName, boolean flipped) {
        limitSwitch = FTCUtilities.getTouchSensor(deviceName);
        this.flipped = flipped;
    }

    @Override
    public boolean isTriggered() {
        //
        return flipped ^ limitSwitch.isPressed();
    }
}
