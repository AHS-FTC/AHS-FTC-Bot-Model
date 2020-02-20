package edu.ahs.robotics.util;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.io.*;
import java.util.ArrayList;
import java.util.List;

import edu.ahs.robotics.control.Point;
import edu.ahs.robotics.util.ftc.FTCUtilities;

import static java.lang.Double.valueOf;

//Directory will need to be: System.getProperty("user.dir"). Will put in working file (use when running mock tests)
    //Directory will need to be: Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS).toString()
    //(when actually running on robot and have phone plugged in)

public class GCodeReader {
    private static GCodeReader instance;

    public static List<List<Point>> openFile(String name){
        BufferedReader fr;
        String fileName = FTCUtilities.getLogDirectory() + "/" + name;
        List<List<Point>> arrayOfPoints = new ArrayList<>();

        try {
            fr = new BufferedReader(new FileReader(fileName));
            String line;
            boolean inPath = false;
            List<Point> points = null;
            while ((line = fr.readLine()) != null){
                String[] stringCoords = line.split(";");
                if (valueOf(stringCoords[2]) == 0.0) {
                    if (!inPath) {
                        points = new ArrayList<>();
                        inPath = true;
                    }
                    Point p = new Point(valueOf(stringCoords[0]), valueOf(stringCoords[1]));
                    points.add(p);
                } else {
                    if (inPath){
                        arrayOfPoints.add(points);
                        inPath = false;
                    }
                }
            }
        } catch (IOException e){
            throw new Warning("Looking for path "+name+" threw exception"+e.getMessage());
        }
        return arrayOfPoints;
    }

}
