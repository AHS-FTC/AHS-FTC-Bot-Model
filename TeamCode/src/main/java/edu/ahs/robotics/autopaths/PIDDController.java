package edu.ahs.robotics.autopaths;

import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

import edu.ahs.robotics.util.FTCUtilities;
import edu.ahs.robotics.util.Logger;

public class PIDDController {

        private double MAXADJUSTMENT = .02;
        private double KP;
        private double KI;
        private double KD;
        private double KDD;
        private double currentError=0;
        private double totalError=0;
        private double changeInError=0;
        private List errorList;


        public PIDDController(double KP, double KI, double KD, double KDD){
            this.KP = KP;
            this.KI = KI;
            this.KD = KD;
            this.KDD = KDD;
            this.errorList = new ArrayList<Double>();
        }

        public double getPowerAdjustment(double error, double deltaTime, double secondDerivative){
            errorList.add(error);
            if (deltaTime==0){
                deltaTime=1;}

            changeInError=(error-currentError)/deltaTime;
            currentError=error;
            totalError+=currentError*deltaTime;
            double pAdjustment = error*KP;
            double iAdjustment = Range.clip(totalError*KI,-MAXADJUSTMENT,MAXADJUSTMENT);
            totalError=Range.clip(totalError,-MAXADJUSTMENT/KI,MAXADJUSTMENT/KI);
            double dAdjustment = changeInError*KD;
            double ddAdjustment = secondDerivative*KDD;

            //Logger.append(Logger.Cats.PADJUSTMENT,Double.toString(pAdjustment));
            //Logger.append(Logger.Cats.IADJUSTMENT,Double.toString(iAdjustment));
            //Logger.append(Logger.Cats.DADJUSTMENT,Double.toString(dAdjustment));
            //Logger.append(Logger.Cats.DDADJUSTMENT,Double.toString(ddAdjustment));

            double totalAdjustment = ddAdjustment+pAdjustment+iAdjustment+dAdjustment;

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

