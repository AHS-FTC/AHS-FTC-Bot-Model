package edu.ahs.robotics;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

public class Logger {

    /*
    INSTRUCTIONS FOR USING THE LOGGER:

    SETUP:
    In opmode, Logger.getInstance().createCats("Feild1","Feild2");
    Can add as many feilds as needed

    ADD DATA:
    In any file, use Logger.append("Feild1",Data);
    Will add data to given feild, not much more than that

    IMPORTANT: make sure that your data is converted to a string before appending
    you can do this in the append function, by using Integer.toString(int) or Double.toString(double)

    FINALIZE AND WRITE FILE:
    Use function logger.getinstance().writeToFile();
    Make sure to write this after your robot.execute();
    Otherwise, file will never be written and data is lost

    You can change names in logger class, default is "Data.csv"

    Directory will by standard be
    Make sure to save file or change name in between runs so that your file is not overwritten
    IF THIS IS NOT DONE DATA WILL BE LOST
     */

    private static Logger instance;
    private Map<String, ArrayList<String>> entriesByCategory;

    private String directory = System.getProperty("user.dir");
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
    }
    public void creatCats(String... categories){
        for (int i = 0; i < categories.length; i++) {
            entriesByCategory.put(categories[i], new ArrayList<String>());
        }
    }


    public void writeToFile() {
        try {
            File file = new File(directory, fileName);
            if (file.exists()) {
                file.delete();
            }
            file.createNewFile();
            csvWriter = new FileWriter(file);
            for (Map.Entry<String, ArrayList<String>> s : entriesByCategory.entrySet()) {
                csvWriter.append(s.getKey());
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

    public static void append(String title, String data) {
        getInstance().entriesByCategory.get(title).add(data);
    }
}
