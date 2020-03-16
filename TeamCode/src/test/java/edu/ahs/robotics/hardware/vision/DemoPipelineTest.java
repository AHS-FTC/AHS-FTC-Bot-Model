package edu.ahs.robotics.hardware.vision;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Environment;

import org.junit.Test;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;


import edu.ahs.robotics.util.ftc.FTCUtilities;

import static org.junit.Assert.*;

public class DemoPipelineTest {


    @Test
    public void testShapes() {
        String fileName = "C:\\FTC\\Code\\vision pics\\shapes_and_colors";
        //String fileName = Environment.DIRECTORY_DOWNLOADS;

        System.out.println(fileName);

        Mat m = Imgcodecs.imread(fileName);

        Imgcodecs.imwrite(fileName + "_new", m);

    }
}