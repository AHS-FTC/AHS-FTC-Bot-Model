package edu.ahs.robotics;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class PIDController {

        private double KP;
        private double KI;
        private double KD;
        private double currentError=0;
        private double totalError=0;
        private double changeInError=0;
        private List errorList;
        private double deltaTime;


        public PIDController(double KP, double KI, double KD){
            this.KP=KP;
            this.KI=KI;
            this.KD=KD;
            this.errorList = new ArrayList<Double>();
        }

        public double getPowerAdjustment(double error, double deltaTime){
            errorList.add(error);
            changeInError=(error-currentError)/deltaTime;
            currentError=error;
            totalError+=currentError*deltaTime;
            double pAdjustment = error*KP;
            double iAdjustment = totalError*KI;
            double dAdjustment = changeInError*KD;

            double totalAdjustment = pAdjustment+iAdjustment+dAdjustment;

            return totalAdjustment;

        }

        public void getMeanAndSD(){
            double meanError = totalError/errorList.size();
            double sd = sd((ArrayList<Double>) errorList);
            FTCUtilities.OpLogger("Mean/SD",Double.toString(meanError)+", "+Double.toString(sd));

        }

    public double sd (ArrayList<Double> errors)
    {
        // Step 1:
        double mean = totalError/errors.size();
        double temp = 0;

        for (int i = 0; i < errors.size(); i++)
        {
            double val = errors.get(i);

            // Step 2:
            double squreDiffToMean = Math.pow(val - mean, 2);

            // Step 3:
            temp += squreDiffToMean;
        }

        // Step 4:
        double meanOfDiffs =  temp / (double) (errors.size());

        // Step 5:
        return Math.sqrt(meanOfDiffs);
    }



}

