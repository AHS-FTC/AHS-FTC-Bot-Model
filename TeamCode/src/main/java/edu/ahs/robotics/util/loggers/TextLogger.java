package edu.ahs.robotics.util.loggers;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import edu.ahs.robotics.util.FTCUtilities;

public class TextLogger extends Logger {
    public TextLogger(String fileName, String key) {
        super(fileName, key, ".txt");
    }

    public void logLine(String text) {
        long currentTime = FTCUtilities.getCurrentTimeMillis() - startTime;

        try {
            fileWriter.write(currentTime + "ms: " + text + "\n");
        } catch (Exception e) {
            throw new Warning("logLine method in textLogger threw an IOException: " + e.getMessage());
        }
    }
}
