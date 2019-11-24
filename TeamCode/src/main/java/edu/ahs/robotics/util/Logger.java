package edu.ahs.robotics.util;

import android.graphics.Bitmap;
import android.os.Environment;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

public class Logger {

    /*
    INSTRUCTIONS FOR USING THE LOGGER:

    SETUP:
    None needed

    ADD DATA:
    In any file, use Logger.append(Logger.cats.(NAME),Data);
    Will add data to given field, not much more than that

    IMPORTANT: make sure that your data is converted to a string before appending
    you can do this in the append function, by using Integer.toString(int) or Double.toString(double)

    FINALIZE AND WRITE FILE:
    Use function logger.getinstance().writeToFile();
    Make sure to write this after your robot.executePlan();
    Otherwise, file will never be written and data is lost

    You can change names in logger class, default is "Data.csv"

    Directory will need to be: System.getProperty("user.dir"). Will put in working file (use when running mock tests)
    Directory will need to be: Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS).toString()
     (when actually running on robot and have phone plugged in)

    Make sure to save file or change name in between runs so that your file is not overwritten
    IF THIS IS NOT DONE DATA WILL BE LOST
     */

    private static Logger instance;
    private Map<Cats, ArrayList<String>> entriesByCategory;

    public static enum Cats {
        MOTORPOW("Motor Power"), ENCODERDIST("Encoder Distance"),DESIDIST("Desired Distance"),ERROR("Error"),
        POWADJ("Power Adjustment"),TIME("Time"),PADJUSTMENT("P Adjustment"),IADJUSTMENT("I Adjustment"),DADJUSTMENT("D Adjustment"),DDADJUSTMENT("DD Adjustment");

        private String name;

        private String getName(){return name;}

        private Cats(String s) { this.name = s;}
    }

    private String fileName = "Data.csv";

    static {
        instance = new Logger();
    }

    private FileWriter csvWriter = null;

    public static Logger getInstance() {
        return instance;
    }

    private Logger() {
        entriesByCategory = new HashMap<>();
        for (int i = 0; i < Cats.values().length; i++) {
            entriesByCategory.put(Cats.values()[i], new ArrayList<String>());

        }
    }

    public void writeToFile() {
        try {
            File file = new File(FTCUtilities.getLogDirectory(), fileName);
            if (file.exists()) {
                file.delete();
            }
            file.createNewFile();
            csvWriter = new FileWriter(file);
            for (Map.Entry<Cats, ArrayList<String>> s : entriesByCategory.entrySet()) {
                csvWriter.append(s.getKey().getName());
                csvWriter.append(", ");
                ArrayList<String> list = s.getValue();
                Iterator<String> iterator = list.iterator();
                while (iterator.hasNext()) {
                    csvWriter.append(iterator.next());
                    if (iterator.hasNext()) {
                        csvWriter.append(", ");
                    }
                }

                csvWriter.append("\n");

            }
            csvWriter.flush();
            csvWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

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
            e.printStackTrace();
        }

    }

    public static void append(Cats title, String data) {
        getInstance().entriesByCategory.get(title).add(data);
    }
}
