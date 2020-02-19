package edu.ahs.robotics.util.ftc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;

import java.util.Calendar;
import java.util.Locale;

import edu.ahs.robotics.util.loggers.TextLogger;

/**
 * Class holding static methods that steal Throwables from the deathly jaws of the FTCApp, logging the StackTrace and other info to a file and not spewing out useless info to the phone screen
 * @author Alex Appleby
 */
public class ErrorStealer {
    private static TextLogger errorLogger;

    /**
     * Sets the logger that a Throwable will log to.
     */
    public static void setErrorLogger(TextLogger errorLogger){
        ErrorStealer.errorLogger = errorLogger;
    }

    /**
     * Takes a throwable, decomposes it's information, then logs it to a file specified by setErrorLogger().
     * If a logger is unspecified, it creates it's own.
     * Method automatically transposes error into FTCApp Warning, throwing that error to the phone screen.
     * Best used in catch block of a try/catch, but also built into our base OpMode
     */
    public static void stealError(Throwable t){
        if(t instanceof Warning){ //if throwable is already ftc compatible, then just throw a standard warning
            throw (Warning)t;
        }

        if(errorLogger == null){ //make a logger if one isn't specified
            errorLogger = new TextLogger("ERROR_LOG", "error");
        }

        errorLogger.logLine("\n \n ----------------------------------- THROWABLE STOPPED EXECUTION: " + t.getClass().getSimpleName()); // add formatting

        errorLogger.logLine("Error Identifier: " + composeErrorIdentifier());

        errorLogger.logLine("Error Message: " + t.getMessage());

        StackTraceElement[] stackTrace = t.getStackTrace();

        int i = 0;

        for(StackTraceElement e : stackTrace){
            errorLogger.logLine("Stack Trace level " + i + ": " + e.toString());
            i++;
        }

        String lowestStackTrace = stackTrace[0].toString(); //get the lowest level of the stack trace for the phone screen

        String outputFile = errorLogger.getOutputFile();

        errorLogger.stopWriting();

        transposeError(t.getMessage(), lowestStackTrace, outputFile);
    }

    private static void transposeError(String errorMessage, String lowestStackTrace, String outputFile){
        throw new Warning("ErrorStealer caught a throwable. It reads: " + errorMessage + " The lowest stack trace is: " + lowestStackTrace + ". You can find more details in " + outputFile);
    }

    /**
     * Makes an 'errorIdentifier' that contains date, time, and OpMode information.
     */
    private static String composeErrorIdentifier(){
        String errorIdentifier;

        Calendar calendar = Calendar.getInstance();

        Locale locale = Locale.getDefault();

        OpMode opMode = FTCUtilities.getOpMode();

        if(opMode == null){
            errorIdentifier = "unspecified OpMode (Not specified by FTCUtilities.setOpMode) ";
        } else {
            errorIdentifier = opMode.getClass().getSimpleName() + " ";
        }

        errorIdentifier += "on ";

        errorIdentifier += calendar.getDisplayName(Calendar.MONTH, Calendar.LONG, locale); //add month

        errorIdentifier += " "; //add a space

        errorIdentifier += calendar.get(Calendar.DAY_OF_MONTH); //add day

        errorIdentifier += ", "; //add a space and comma

        errorIdentifier += calendar.get(Calendar.HOUR_OF_DAY); //add hour

        errorIdentifier += ":"; //hour/minute formatting

        errorIdentifier += calendar.get(Calendar.MINUTE); //add minute

        errorIdentifier += ":"; //min/sec formatting

        errorIdentifier += calendar.get(Calendar.SECOND); //add second

        return errorIdentifier;
    }

}
