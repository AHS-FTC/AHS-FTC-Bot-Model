package edu.ahs.robotics;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

public class FTCUtilities { //handles inaccessable objects in FTCapp. hardwareMap exists under OpMode.
    private static HardwareMap hardwareMap;
    private static OpMode opMode;

    private static boolean testMode = false;
    private static Map <String, DcMotor>testMotors = new HashMap();

    public static void setHardwareMap(HardwareMap hardwareMap) {
        FTCUtilities.hardwareMap = hardwareMap;
    }

    public static HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public static void setOpMode(OpMode opMode){
        FTCUtilities.opMode = opMode;
    }

    public static OpMode getOpMode(){
        return opMode;
    }

    public static void OpLogger(String caption, Object object){
        if(!testMode) {
            opMode.telemetry.addData(caption, object);
            opMode.telemetry.update();
        } else {
            System.out.println(caption + ": " + object);
        }
    }

    public static void OpSleep(long ms) {
        if(testMode){
            try {
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        } else if(opMode instanceof LinearOpMode){
            LinearOpMode linearOpMode = (LinearOpMode)opMode;
            linearOpMode.sleep(ms);
        }
    }

    public static DcMotor getMotor(String deviceName) {
        if(testMode){
            return testMotors.get(deviceName);
        }
        return hardwareMap.get(DcMotor.class, deviceName);
    }

    static void addTestMotor(DcMotor motor, String deviceName){
        if(!testMode){
            throw new UnsupportedOperationException("Not in testMode! Make sure to call startTestMode first");
        }
        testMotors.put(deviceName, motor);
    }

    public static void startTestMode(){
        testMode = true;
    }

    private FTCUtilities () {} //no constructo statico
}
