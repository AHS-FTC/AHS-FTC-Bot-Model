package edu.ahs.robotics.hardware.vision;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import edu.ahs.robotics.hardware.sensors.IMU;

public class DemoPipeline extends OpenCvPipeline {

    private Mat grayscale = new Mat();
    private Mat blurred = new Mat();
    private Mat blackAndWhite = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,grayscale,Imgproc.COLOR_BGR2GRAY);
        Imgproc.blur(grayscale,blurred, new Size(5,5));
        Imgproc.threshold(blurred,blackAndWhite,60, 255,Imgproc.THRESH_BINARY);

        return blackAndWhite;
    }
}
