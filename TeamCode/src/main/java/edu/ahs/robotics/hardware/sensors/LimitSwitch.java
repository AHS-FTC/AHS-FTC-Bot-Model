package edu.ahs.robotics.hardware.sensors;

import com.qualcomm.robotcore.hardware.TouchSensor;

import edu.ahs.robotics.util.FTCUtilities;

public class LimitSwitch implements Trigger {
    TouchSensor limitSwitch;

    public LimitSwitch(String deviceName) {
        limitSwitch = FTCUtilities.getTouchSensor(deviceName);
    }

    @Override
    public boolean isTriggered() {
        return limitSwitch.isPressed();
    }
}
