package edu.ahs.robotics.util.opmodes.bfr;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.HashMap;

import edu.ahs.robotics.util.ftc.ErrorStealer;
import edu.ahs.robotics.util.ftc.FTCUtilities;
import edu.ahs.robotics.util.ftc.LoopTracker;
import edu.ahs.robotics.util.loggers.TextLogger;

/**
 * 16896's own OpMode class that automatically steals loose errors and sets the FTCUtilities OpMode.
 * Also establishes mainLogger, an instance of TextLogger for general use and stolen error output.
 * To use, create a subclass and override methods with your own code.
 *
 * @see LinearOpMode16896
 */
public abstract class IterativeOpMode16896 extends OpMode {
    protected TextLogger mainLogger;
    protected LoopTracker loopTracker;

    @Override
    public final void init() {
        mainLogger = new TextLogger("iterativeOpModeMain", "main");
        mainLogger.startWriting();
        ErrorStealer.setErrorLogger(mainLogger);
        FTCUtilities.setOpMode(this);

        loopTracker = new LoopTracker();

        try {
            initialize();
        } catch (Throwable t) {
            ErrorStealer.stealError(t);
        }

        mainLogger.logLine("Initialization Finished");
    }

    @Override
    public final void init_loop() {
        try {
            repeatAfterInit();
        } catch (Throwable t) {
            ErrorStealer.stealError(t);
        }
    }

    @Override
    public final void start() {
        mainLogger.logLine("Start Was Pressed");
        try {
            begin();
        } catch (Throwable t) {
            ErrorStealer.stealError(t);
        }
        mainLogger.logLine("Start Code Finished");
    }

    @Override
    public final void loop() {
        try {
            iterate();

            loopTracker.update();
        } catch (Throwable t) {
            ErrorStealer.stealError(t);
        }
    }

    @Override
    public final void stop() {
        try {
            teardown();
        } catch (Throwable t) {
            ErrorStealer.stealError(t);
        }

        mainLogger.logLine("---LOOP TIME STATS:---");
        mainLogger.logLine("Max DeltaTime: " + loopTracker.getMaxDeltaTime() + "ms");
        mainLogger.logLine("Mean DeltaTime: " + loopTracker.getAverageDeltaTime() + "ms");

        mainLogger.logLine("DeltaTime Classifications: \n" + getDeltaTimeClassifications());
        mainLogger.logLine("Teardown Finished");
        mainLogger.stopWriting();
    }

    @SuppressWarnings("all") //todo ask john
    private String getDeltaTimeClassifications() {
        HashMap<LoopTracker.Categories, Double> timePercentages = loopTracker.getTimePercentages();

        return (
                "--- 0-10ms: " + timePercentages.get(LoopTracker.Categories.MS_0_10) * 100 + "% \n" +
                        "--- 11-30ms: " + timePercentages.get(LoopTracker.Categories.MS_11_30) * 100 + "% \n" +
                        "--- 31-50ms: " + timePercentages.get(LoopTracker.Categories.MS_31_50) * 100 + "% \n" +
                        "--- 51-70ms: " + timePercentages.get(LoopTracker.Categories.MS_51_70) * 100 + "% \n" +
                        "--- 71-90ms: " + timePercentages.get(LoopTracker.Categories.MS_71_90) * 100 + "% \n" +
                        "--- 91-120ms: " + timePercentages.get(LoopTracker.Categories.MS_91_120) * 100 + "% \n" +
                        "--- 120+ms: " + timePercentages.get(LoopTracker.Categories.MS_120PLUS) * 100 + "% \n"
        );
    }

    /**
     * Override in place of init();
     */
    protected abstract void initialize(); //Override this!

    /**
     * Override in place of init_loop();
     */
    protected abstract void repeatAfterInit(); //Override this!

    /**
     * Override in place of start();
     */
    protected abstract void begin(); //Override this!

    /**
     * Override in place of loop();
     */
    protected abstract void iterate(); //Override this!

    /**
     * Override in place of stop();
     */
    protected abstract void teardown(); //Override this!
}
