package edu.ahs.robotics.util.ftc;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.io.File;
import java.io.FileOutputStream;
import java.util.Calendar;
import java.util.HashMap;
import java.util.Map;

import edu.ahs.robotics.hardware.sensors.Odometer;
import edu.ahs.robotics.hardware.sensors.OdometerImpl;
import edu.ahs.robotics.util.MockClock;
import edu.ahs.robotics.util.ParameterLookup;

/**
 * General Utilities class to manage the crappy FTC classes
 * Enables Mocking via the get-InsertHardwareDeviceHere- methods and the testMode boolean
 * @author Alex Appleby
 */
public class FTCUtilities { //handles inaccessable objects in FTCApp. hardwareMap exists under OpMode.
    private static HardwareMap hardwareMap;
    private static OpMode opMode;

    private static boolean testMode = false;
    private static Map <String, DcMotor>testMotors = new HashMap();
    private static Map <String, Odometer>testOdometers = new HashMap();
    private static MockClock mockClock;
    private static ParameterLookup parameterLookup;


    public static String getLogDirectory(){
        if (testMode){
            return (System.getProperty("user.dir"));
        } else{
            return (Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS).toString());
        }
    }

    public static HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public static void setOpMode(OpMode opMode){
        FTCUtilities.opMode = opMode;
        FTCUtilities.hardwareMap = opMode.hardwareMap;
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

    public static void addData(String caption, Object object){

        if(!testMode) {
            opMode.telemetry.addData(caption, object);
        } else {
            System.out.println(caption + ": " + object);
        }
    }

    public static void addLine(String line){
        if(!testMode) {
            opMode.telemetry.addLine(line);
        } else {
            System.out.println(line);
        }
    }

    public static void updateOpLogger(){
        if(!testMode){
            opMode.telemetry.update();
        }
    }

    public static void sleep(long ms) {
            try {
                Thread.sleep(ms);
            } catch (InterruptedException e) {
                throw new Warning("Died in FTCUtilities sleep");
            }

    }

    public static DcMotor getMotor(String deviceName) {
        if(testMode){
            return testMotors.get(deviceName);
        }
        return hardwareMap.get(DcMotor.class, deviceName);
    }

    public static BNO055IMU getIMU (String imuName){
        if(testMode){
            return null;//literally die in a hole // todo mock imu
        }
        return hardwareMap.get(BNO055IMU.class, imuName);
    }

    public static TouchSensor getTouchSensor (String sensorName){
        if(testMode){
            throw new UnsupportedOperationException("TestMode doesn't support touch sensors yet. mock it");
        } else {
            return hardwareMap.get(TouchSensor.class, sensorName);
        }
    }

    public static Rev2mDistanceSensor getDistanceSensor(String sensorName){
        if(!testMode) {
            Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, sensorName);
            return distanceSensor;
        } else {
            throw new UnsupportedOperationException("TestMode doesn't support distance sensors yet. mock it");
        }
    }

    public static void addTestMotor(DcMotor motor, String deviceName){
        if(!testMode){
            throw new UnsupportedOperationException("Not in testMode! Make sure to call startTestMode first");
        }
        testMotors.put(deviceName, motor);
    }

    public static void addTestOdometer(Odometer odometer, String deviceName){
        if(!testMode){
            throw new UnsupportedOperationException("Not in testMode! Make sure to call startTestMode first");
        }
        testOdometers.put(deviceName, odometer);
    }

    public static Servo getSerialServo(String deviceName){
        if(!testMode){
            return hardwareMap.get(Servo.class, deviceName);
        } else {
            throw new Error("TestMode doesn't support servos yet. mock it");
        }
    }

    public static CRServo getCRServo(String deviceName){
        if(!testMode){
            return hardwareMap.get(CRServo.class, deviceName);
        } else {
            throw new Error("TestMode doesn't support servos yet. mock it");
        }
    }

    public static Odometer getOdometer(String deviceName, double wheelDiameter, boolean flip, double ticksPerRotation){
        if(testMode){
            return testOdometers.get(deviceName);
        } else {
            return new OdometerImpl(deviceName,wheelDiameter,flip, ticksPerRotation);
        }
    }

    public static boolean opModeIsActive(){
        if(opMode instanceof LinearOpMode){
            return ((LinearOpMode) opMode).opModeIsActive();
        } else {
            return true;
        }
    }

    public static long getCurrentTimeMillis(){
        if(!testMode){
            return System.currentTimeMillis();
        } else {
            return mockClock.getCurrentTimeMillis();
        }
    }

    public static ParameterLookup getParameterLookup() {
        return parameterLookup;
    }

    public static void setParameterLookup(ParameterLookup parameterLookup) {
        FTCUtilities.parameterLookup = parameterLookup;
    }

    /**
     * Saves a bitmap to the phone. Sets the file name using time & date information.
     * Saves can be found in the downloads folder of the phone.
     */
    public static void saveImage(Bitmap bitmap){
        Calendar now = Calendar.getInstance();
        String filePath = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS).toString();
        String fileName = "BotImg_"+ now.get(Calendar.DAY_OF_MONTH) + "_" + now.get(Calendar.HOUR_OF_DAY) + "_" + now.get(Calendar.MINUTE) + "_" + now.get(Calendar.SECOND) + now.get(Calendar.MILLISECOND) +".jpg";
        File img = new File(filePath, fileName);
        if (img.exists())
            img.delete();
        try {
            FileOutputStream out = new FileOutputStream(img);
            bitmap.compress(Bitmap.CompressFormat.JPEG, 100, out);
            out.flush();
            out.close();


        } catch (Exception e) {
            throw new Warning(e.getMessage());
        }
    }

    public static void setMockClock(MockClock mockClock){
        FTCUtilities.mockClock = mockClock;
    }

    public static void startTestMode(){
        testMode = true;
    }

    private FTCUtilities () {} //no constructo statico


//    public static class MockableGamePad {
//    private static String lastInput;
//    private static Scanner input = new Scanner(System.in);
//
//    public static double rightTrigger(){
//        if(testMode){
//            getInput();
//            if(lastInput.contains("rt")){
//                return 1;
//            } else if (lastInput.contains("RT")){
//                return -1;
//            } else {
//                return 0;
//            }
//        } else {
//            return opMode.gamepad1.right_trigger;
//        }
//    }
//
//
//
//        private static void getInput(){
//            lastInput = input.nextLine();
//        }
//    }

}
