package edu.ahs.robotics.hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import edu.ahs.robotics.autocommands.PlanElement;
import edu.ahs.robotics.autocommands.obmcommands.ServoCommand;
import edu.ahs.robotics.util.FTCUtilities;

public class SerialServo implements Executor {
    private Servo servo;

    public SerialServo(String deviceName, boolean reverse) {
        servo = FTCUtilities.getServo(deviceName);
        if(reverse){
            servo.setDirection(Servo.Direction.REVERSE);
        } else {
            servo.setDirection(Servo.Direction.FORWARD);
        }
    }

    @Override
    public void execute(PlanElement planElement) {
        if(planElement instanceof ServoCommand){
            setPosition(((ServoCommand) planElement).position);
        } else {
            throw new Warning("Could not execute PlanElement " + planElement.toString() + " in SerialServo class");
        }
    }

    public void setPosition(double position){
        servo.setPosition(position);
    }
}
