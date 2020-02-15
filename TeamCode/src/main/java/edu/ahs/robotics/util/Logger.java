package edu.ahs.robotics.util;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Iterator;

public abstract class Logger {
    private String fileName;
    protected static long startTime; //shared across all loggers for synchronization
    protected boolean firstLine;
    private static HashMap<String, Logger> loggers = new HashMap<>();

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

    protected FileWriter fileWriter = null;

    public void startWriting() {
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

    public void stopWriting() {
        try{
            fileWriter.close();
        } catch (IOException e) {
            throw new Warning(e.getMessage());
        }
    }
}
