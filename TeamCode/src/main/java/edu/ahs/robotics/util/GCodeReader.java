package edu.ahs.robotics.util;

import android.os.Environment;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.io.*;
import java.util.ArrayList;
import edu.ahs.robotics.autocommands.autopaths.functions.Position;
import edu.ahs.robotics.hardware.Path;

import static java.lang.Double.valueOf;

//Directory will need to be: System.getProperty("user.dir"). Will put in working file (use when running mock tests)
    //Directory will need to be: Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS).toString()
    //(when actually running on robot and have phone plugged in)

public class GCodeReader {
    private static GCodeReader instance;

    public static ArrayList<Path.Point> openFile(String name){
        BufferedReader fr;
        String fileName = FTCUtilities.getLogDirectory() + "/" + name;
        ArrayList<Path.Point> points = new ArrayList<>();

        try {
            fr = new BufferedReader(new FileReader(fileName));
            String line;
            while ((line = fr.readLine()) != null){
                String[] stringCoords =line.split(";");
                Path.Point p = new Path.Point(valueOf(stringCoords[0]), valueOf(stringCoords[1]));
                points.add(p);
            }
        } catch (IOException e){
            throw new Warning("Looking for path "+name+" threw exception"+e.getMessage());
        }
        return points;
    }

}
