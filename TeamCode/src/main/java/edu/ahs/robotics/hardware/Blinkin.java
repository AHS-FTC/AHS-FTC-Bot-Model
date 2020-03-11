package edu.ahs.robotics.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Servo;

import edu.ahs.robotics.util.ftc.FTCUtilities;

public class Blinkin {
    private RevBlinkinLedDriver blinkin;
    private RevBlinkinLedDriver.BlinkinPattern lastPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;

    public Blinkin(String deviceName) {
        blinkin = FTCUtilities.getBlinkin(deviceName);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern){
        if (pattern == lastPattern) {
            return;
        } else {
            blinkin.setPattern(pattern);
            lastPattern = pattern;
        }
    }

}
