package edu.ahs.robotics;

import java.util.ArrayList;
import java.util.Collection;

public class PIDController {

        private double KP;
        private double KI;
        private double KD;
        private double currentError;
        private double totalError;
        private double changeInError;


        public PIDController(double KP, double KI, double KD){
            this.KP=KP;
            this.KI=KI;
            this.KD=KD;
            this.currentError=0;
            totalError=0;
            changeInError=0;
        }

        public double getPowerAdjustment(double error){
            setChangeInError(error);
            updateTotalError(error);
            double pAdjustment = error*KP;
            double iAdjustment = totalError*KI;
            double dAdjustment = changeInError*KD;

            double totalAdjustment = pAdjustment+iAdjustment+dAdjustment;

            return totalAdjustment;

        }

        private void setChangeInError(double error){
            changeInError=error-currentError;
            setCurrentError(error);
        }

        private void setCurrentError(double error){
            currentError=error;
        }

        private void updateTotalError(double currentError){
            totalError+=currentError;
        }

}

