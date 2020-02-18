package edu.ahs.robotics.util.loggers;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.io.IOException;

import edu.ahs.robotics.util.FTCUtilities;

/**
 * Logger class that logs bulk string information to a .txt file. Logs are timestamped with program runtime.
 * TextLogger automatically starts itself, calling Logger's startWriting() in the constructor.
 * 16896 OpModes incorporate a 'main logger' for general use
 * @see edu.ahs.robotics.util.opmodes.LinearOpMode16896
 * @author Alex Appleby
 */
public class TextLogger extends Logger {
    public TextLogger(String fileName, String key) {
        super(fileName, key, ".txt");
        super.startWriting();//textLogger automatically starts itself
        logLine("logger initialized and started");
    }

    @Override
    public void startWriting() { //todo ask john if there's a better way to prevent use here
        //do nothing
    }

    public void logLine(String text) {
        long currentTime = FTCUtilities.getCurrentTimeMillis() - startTime;

        try {
            fileWriter.write(currentTime + "ms: " + text + "\n");
        } catch (IOException e) {
            throw new Warning("logLine method in textLogger threw an IOException: " + e.getMessage());
        }
    }
}
