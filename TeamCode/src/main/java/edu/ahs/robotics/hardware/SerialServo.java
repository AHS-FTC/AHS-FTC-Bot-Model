package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import edu.ahs.robotics.util.FTCUtilities;

public class SerialServo {
    private Servo servo;

    public SerialServo(String deviceName, boolean reverse) {
        servo = FTCUtilities.getServo(deviceName);
        if(reverse){
            servo.setDirection(Servo.Direction.REVERSE);
        } else {
            servo.setDirection(Servo.Direction.FORWARD);
        }
    }

    public void setPosition(double position){
        servo.setPosition(position);
    }
}
