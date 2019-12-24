package edu.ahs.robotics.hardware.sensors;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import edu.ahs.robotics.util.FTCUtilities;

public class Vuforia {
    private VuforiaLocalizer vuforiaLocalizer;


    public Vuforia() {
        OpMode opMode = FTCUtilities.getOpMode();
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId"," id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AbuxcJX/////AAABmXadAYnA80uwmb4Rhy4YmvIh7qg/f2yrRu1Nd8O7sSufbUWHSv1jDhunwDBItvFchrvkc8EjTzjh97m2kAPy8YOjBclQbEBtuR8qcIfrGofASCZh2M6vQ0/Au+YbhYh0MLLdNrond+3YjkLswv6+Se3eVGw9y9fPGamiABzIrosjUdanAOWemf8BtuQUW7EqXa4mNPtQ+2jpZQO2sqtqxGu1anHQCD0S/PvdZdB7dRkyWaH6XTZCat5gZ0fpFH/aLWMFP4yiknlgYbjT7gklUAqyDX81pNrQhWWY4dOFnz2WiWhkCt+MNZMLKH5SdsyC7gwKI/r3h51pTwgXZfyYymB60eYAFqEUpeTrL+4LmltN";

        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(params);
        vuforiaLocalizer.enableConvertFrameToBitmap();
        vuforiaLocalizer.setFrameQueueCapacity(1);
    }

    /**
     * Grabs a bitmap from the Vuforia engine for image processing
     * @return camera output as a bitmap
     */
    public Bitmap getBitmap(){
        Bitmap bitmap;
        VuforiaLocalizer.CloseableFrame frame;

        try {
            frame = vuforiaLocalizer.getFrameQueue().take();
        }catch (Exception e) {
            throw new Warning("couldn't find vuforia frame");
        }

        bitmap = vuforiaLocalizer.convertFrameToBitmap(frame);
        return bitmap;
    }

}
