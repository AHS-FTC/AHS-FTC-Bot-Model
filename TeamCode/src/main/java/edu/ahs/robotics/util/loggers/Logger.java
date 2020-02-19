package edu.ahs.robotics.util.loggers;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Iterator;

import edu.ahs.robotics.util.ftc.FTCUtilities;

public abstract class Logger {
    private String fileName;
    protected static long startTime; //shared across all loggers for synchronization
    protected boolean firstLine;
    private boolean writing = false;
    private static HashMap<String, Logger> loggers = new HashMap<>();
    protected FileWriter fileWriter = null;

    static {
        startTime = System.currentTimeMillis();
    }

    public Logger(String fileName, String key, String fileType){
        this.fileName = fileName + fileType;
        loggers.put(key, this);
    }

    public static Logger getLogger(String key){
        return loggers.get(key);
    }

    public static void stopLoggers(){
        for (Iterator iterator = loggers.values().iterator(); iterator.hasNext(); ) {
            Logger next =  (Logger) iterator.next();
            next.stopWriting();
        }
    }


    public void startWriting() {
        writing = true;
        firstLine = true;
        try {
            File file = new File(FTCUtilities.getLogDirectory(), fileName);
            if (file.exists()) {
                file.delete();
            }
            file.createNewFile();
            fileWriter = new FileWriter(file);
            fileWriter.flush();

         } catch (IOException e) {
            throw new Warning(e.getMessage());
        }
    }

    /**
     * @return the name of the file that this Logger is logging to
     */
    public String getOutputFile(){
        return fileName;
    }

    public boolean isWriting(){
        return writing;
    }

    public void stopWriting() {
        writing = false;
        try{
            fileWriter.close();
        } catch (IOException e) {
            throw new Warning(e.getMessage());
        }
    }
}
