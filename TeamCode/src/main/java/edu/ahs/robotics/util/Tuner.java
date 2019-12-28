package edu.ahs.robotics.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.HashMap;


/**
 * A util class that enables numerical tuning without redownloading by using gamepad1 inputs.
 * @author Alex Appleby
 */
public class Tuner {
    public static HashMap<Vals,Double> tuningParams; // the actual output mapped to a val enum
    private static boolean running = false;
    private static final double SPEED_CONSTANT   = 0.00001; // the speed at which values change

    /**
     * Enums for reference in a HashMap that work as keys to access tuned values. To be added to at the convenience of the dev.
     */
    public enum Vals {
        P("Proportional"),
        I("Integral"),
        D("Derivative");

        /**
         * The logging tag that's associated with this Val in the logger.
         */
        public String tag;

        Vals(String tag) {
            this.tag = tag;
        }
    }

    /**
     * Runs blocking main loop that logs and checks for gamepad1 inputs to tune. To be ran in init.
     * @param valsList The values that you want to tune. More can be added in the enum if you have something specific.
     */
    public static void start(ArrayList<Vals> valsList){
        tuningParams = new HashMap<>();
        running = true;
        int valsListIndex = 0;
        Gamepad gamepad1 = FTCUtilities.getOpMode().gamepad1;
        Switch upSwitch = new Switch();
        Switch downSwitch = new Switch();


        for(Vals val: valsList){
            tuningParams.put(val,0.0);
        }

        while (running){
            for(Vals val : tuningParams.keySet()){
                String output = "";

                output += val.tag;
                output += ": ";
                output += Double.toString(tuningParams.get(val));
                if(valsList.get(valsListIndex) == val){
                    output += "<---"; // Indicator so that we know which value we're editing
                }

                FTCUtilities.addLine(output);
            }

            if(gamepad1.dpad_down) {
                if (upSwitch.flip()) {
                    if (valsListIndex >= 1) {
                        valsListIndex -= 1;
                    } else {
                        valsListIndex = valsList.size() - 1; //wrap to top
                    }
                }
            } else if(gamepad1.dpad_up){
                if (downSwitch.flip()) {
                    if (valsListIndex < valsList.size() - 1) {
                        valsListIndex += 1;
                    } else {
                        valsListIndex = 0; // wrap around to 0
                    }
                }
            } else {
                Vals currentValKey =  valsList.get(valsListIndex);
                double currentValDouble = tuningParams.get(currentValKey);
                currentValDouble += gamepad1.right_trigger * SPEED_CONSTANT;
                currentValDouble -= gamepad1.left_trigger * SPEED_CONSTANT;
                tuningParams.put(currentValKey, currentValDouble);
            }
            FTCUtilities.updateOpLogger();
        }
    }

    /**
     * Ends main tuner loop while keeping tuning vals in tact and accessible
     */
    public static void stop(){
        running = false;
    }

    //stolen from teleop
    private static class Switch { //todo factor out repeated code with tele
        private final static double BUTTON_THRESHOLD = 300; //in millis - time between presses
        protected long lastPress;

        public Switch() {
            lastPress = System.currentTimeMillis();
        }

        public boolean flip() {
            long time = System.currentTimeMillis();

            if(time - lastPress > BUTTON_THRESHOLD) {
                lastPress = time;
                return true;
            }
            return false;
        }
    }

    private Tuner(){} // no constructo statico

}
