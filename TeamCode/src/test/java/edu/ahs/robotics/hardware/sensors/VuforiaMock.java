package edu.ahs.robotics.hardware.sensors;

import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import org.firstinspires.ftc.teamcode.R;

import edu.ahs.robotics.hardware.sensors.Vuforia;

public class VuforiaMock extends Vuforia {

    @Override
    public Bitmap getBitmap(){
        //BitmapFactory.decodeResource();
        return null;//BitmapFactory.decodeResource();
    }
}
