package edu.ahs.robotics.hardware;

import edu.ahs.robotics.util.ftc.FTCUtilities;

public class ContinuosServo {
    private com.qualcomm.robotcore.hardware.CRServo servo;

    public ContinuosServo(String deviceName) {
        servo = FTCUtilities.getCRServo(deviceName);
    }

    public void setPower(double power){
        servo.setPower(power);
    }
}
