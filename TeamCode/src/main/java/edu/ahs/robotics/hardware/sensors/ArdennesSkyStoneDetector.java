package edu.ahs.robotics.hardware.sensors;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.disnodeteam.dogecv.filters.LeviColorFilter;

import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.Logger;

public class ArdennesSkyStoneDetector {
    private Vuforia vuforia;
    private boolean isImageSavingEnabled;

    //private final int IMAGE_X = 0;
    //private final int IMAGE_Y = 500;
    //private final int IMAGE_WIDTH = 1000;
    //private final int IMAGE_HEIGHT = 200;

    private final int LEFT_X = 90 , LEFT_Y = 580;// LEFT_HEIGHT = 0, LEFT_WIDTH = 0;
    private final int MIDDLE_X = 400 , MIDDLE_Y = 580;// MIDDLE_HEIGHT = 0, MIDDLE_WIDTH = 0;
    private final int RIGHT_X = 700 , RIGHT_Y = 580;// RIGHT_HEIGHT = 0, RIGHT_WIDTH = 0;

    private final int STONE_WIDTH = 100, STONE_HEIGHT = 25;

    private final ColorPreset ACTIVE_YELLOW = ColorPreset.PURE_YELLOW; //Change these bad boys to calibrate
    private final ColorPreset ACTIVE_BLACK = ColorPreset.PURE_BLACK;

    public enum SkyStoneConfigurations {
        ONE_FOUR,
        TWO_FIVE,
        THREE_SIX,
    }

    private enum ColorPreset {
        PURE_YELLOW(255,255,0),
        PURE_BLACK(0,0,0),

        BASEMENT_YELLOW(0,0,0),
        BASEMENT_BLACK(0,0,0);

        int r,g,b;

        ColorPreset(int r, int g, int b){
            this.r = r;
            this.g = g;
            this.b = b;
        }

    }


    public ArdennesSkyStoneDetector(boolean isImageSavingEnabled) {
        vuforia = new Vuforia();
        this.isImageSavingEnabled = isImageSavingEnabled;
    }

    public ArdennesSkyStoneDetector(){
        this(false);
    }

    public SkyStoneConfigurations look(){
        Bitmap vuBitmap = vuforia.getBitmap();
        //Bitmap croppedBitmap = Bitmap.createBitmap(vuBitmap, IMAGE_X, IMAGE_Y, IMAGE_WIDTH, IMAGE_HEIGHT);

        Bitmap stoneLeft = Bitmap.createBitmap(vuBitmap, LEFT_X, LEFT_Y, STONE_WIDTH, STONE_HEIGHT);
        Bitmap stoneMiddle = Bitmap.createBitmap(vuBitmap, MIDDLE_X, MIDDLE_Y, STONE_WIDTH, STONE_HEIGHT);
        Bitmap stoneRight = Bitmap.createBitmap(vuBitmap, RIGHT_X, RIGHT_Y, STONE_WIDTH, STONE_HEIGHT);

        if(isImageSavingEnabled) {
            Logger.saveImage(vuBitmap);
            Logger.saveImage(stoneLeft);
            Logger.saveImage(stoneMiddle);
            Logger.saveImage(stoneRight);
        }
        //Ratio is measured blackness to yellowness. higher ratio is more likeliness to be a skystone.

        double leftRatio = getColorness(stoneLeft, ACTIVE_BLACK)/getColorness(stoneLeft, ACTIVE_YELLOW);
        double middleRatio = getColorness(stoneMiddle, ACTIVE_BLACK)/getColorness(stoneMiddle, ACTIVE_YELLOW);
        double rightRatio = getColorness(stoneRight, ACTIVE_BLACK)/getColorness(stoneRight, ACTIVE_YELLOW);

        FTCUtilities.addData("Left Skystone Ratio", leftRatio);
        FTCUtilities.addData("Middle Skystone Ratio", middleRatio);
        FTCUtilities.addData("Right Skystone Ratio", rightRatio);

        if(leftRatio > middleRatio && leftRatio > rightRatio){
            return SkyStoneConfigurations.ONE_FOUR;
        } else if (middleRatio > leftRatio && middleRatio > leftRatio){
            return SkyStoneConfigurations.TWO_FIVE;
        } else {
            return SkyStoneConfigurations.THREE_SIX;
        }

    }

    private double getColorness(Bitmap bitmap, ColorPreset colorPreset){//finds the closeness of a region to a color
        int color;

        int r, g, b;

        double distanceSum = 0;
        int pixels = bitmap.getWidth()*bitmap.getHeight();

        for(int i = 0; i < bitmap.getWidth(); i++){
            for(int j = 0; j < bitmap.getHeight(); j++) {
                color = bitmap.getPixel(i, j);
                r = Color.red(color);
                g = Color.green(color);
                b = Color.blue(color);
                distanceSum += getColorDistance(r,g,b,colorPreset.r, colorPreset.g, colorPreset.b);//todo refactor
            }
        }

        double averageDistance = distanceSum/pixels;

        if(averageDistance != 0){
            return 1/averageDistance;
        } else {
            return Double.POSITIVE_INFINITY;
        }

        //return 1/averageDistance; //average
    }

    private double getColorDistance(int r, int g, int b, int targetR, int targetG, int targetB){//does the actual mAthS
        int rDifference = r - targetR;
        int gDifference = g - targetG;
        int bDifference = b - targetB;

        int rDifferenceSquared = (int) Math.pow(rDifference, 2);
        int gDifferenceSquared = (int) Math.pow(gDifference, 2);
        int bDifferenceSquared = (int) Math.pow(bDifference, 2);

        int sum = rDifferenceSquared + gDifferenceSquared + bDifferenceSquared;

        double distance = Math.sqrt(sum);

        return distance;
    }
}
