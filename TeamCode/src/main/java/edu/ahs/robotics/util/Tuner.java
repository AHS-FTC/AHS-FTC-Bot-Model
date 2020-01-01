package edu.ahs.robotics.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.HashMap;


/**
 * A util class that enables numerical tuning without redownloading by using gamepad1 inputs.
 * @author Alex Appleby
 */
public class Tuner implements ParameterLookup {
    public static HashMap<String,Double> tuningParams; // the actual output mapped to a val enum
    private static boolean running = false;
    private static final double SPEED_CONSTANT = 0.000001; // the speed at which values change

    /**
     * Runs blocking main loop that logs and checks for gamepad1 inputs to tune. To be ran in init.
     * @param paramNames
     */
    public void start(String... paramNames){
        tuningParams = new HashMap<>();
        running = true;
        int valsListIndex = 0;
        Gamepad gamepad1 = FTCUtilities.getOpMode().gamepad1;
        Switch upSwitch = new Switch();
        Switch downSwitch = new Switch();


        for(String name: paramNames){
            tuningParams.put(name,0.0);
        }

        while (running){
            for(String name : paramNames){
                String output = "";

                output += name;
                output += ": ";
                output += Double.toString(tuningParams.get(name));
                if(paramNames[valsListIndex].equals(name)){
                    output += "<---"; // Indicator so that we know which value we're editing
                }

                FTCUtilities.addLine(output);
            }

            if(gamepad1.dpad_down) {
                if (upSwitch.flip()) {
                    if (valsListIndex >= 1) {
                        valsListIndex -= 1;
                    } else {
                        valsListIndex = paramNames.length - 1; //wrap to top
                    }
                }
            } else if(gamepad1.dpad_up){
                if (downSwitch.flip()) {
                    if (valsListIndex < paramNames.length - 1) {
                        valsListIndex += 1;
                    } else {
                        valsListIndex = 0; // wrap around to 0
                    }
                }
            } else {
                String currentValKey = paramNames[valsListIndex];
                double currentValDouble = tuningParams.get(currentValKey);
                currentValDouble += gamepad1.right_trigger * SPEED_CONSTANT;
                currentValDouble -= gamepad1.left_trigger * SPEED_CONSTANT;
                tuningParams.put(currentValKey, currentValDouble);
            }
            FTCUtilities.updateOpLogger();

            if (gamepad1.x) {
                stop();
            }
        }
    }

    /**
     * Ends main tuner loop while keeping tuning vals intact and accessible
     */
    public void stop(){
        running = false;
    }

    @Override
    public double getParameter(String name) {
        return tuningParams.get(name);
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

    public Tuner(){}

}
