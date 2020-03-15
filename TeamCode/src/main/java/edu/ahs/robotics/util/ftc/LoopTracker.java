package edu.ahs.robotics.util.ftc;

import java.util.HashMap;


public class LoopTracker {

    private long runningSum = 0L;
    private int numberOfDataPoints = 0;
    private double runningAverage; // of loop times
    private long maxDeltaTime = 0L;

    private boolean firstUpdate = true;
    private long lastTime;

    /**
     * Categorization of total loop times
     */
    public enum Categories {
        MS_0_10,
        MS_11_30,
        MS_31_50,
        MS_51_70,
        MS_71_90,
        MS_91_120,
        MS_120PLUS
    }

    private HashMap<Categories,Integer> timeCategorizations;

    public LoopTracker() {
        timeCategorizations = new HashMap<>();
        timeCategorizations.put(Categories.MS_0_10,0);
        timeCategorizations.put(Categories.MS_11_30,0);
        timeCategorizations.put(Categories.MS_31_50,0);
        timeCategorizations.put(Categories.MS_51_70,0);
        timeCategorizations.put(Categories.MS_71_90,0);
        timeCategorizations.put(Categories.MS_91_120,0);
        timeCategorizations.put(Categories.MS_120PLUS,0);

    }

    public void update(){
        if (firstUpdate){
            lastTime = FTCUtilities.getCurrentTimeMillis();

            firstUpdate = false;
            return;
        }

        long deltaTime = FTCUtilities.getCurrentTimeMillis() - lastTime;

        runningSum += deltaTime;
        numberOfDataPoints++;

        runningAverage = ((double) runningSum) / ((double) numberOfDataPoints); //avoid integer division

        if(deltaTime > maxDeltaTime){
            maxDeltaTime = deltaTime;
        }

        if(deltaTime < 10){ //find correct categorization for deltaTime
            incrementHashMap(Categories.MS_0_10);
        } else if (deltaTime < 30){
            incrementHashMap(Categories.MS_11_30);
        } else if (deltaTime < 50){
            incrementHashMap(Categories.MS_31_50);
        } else if (deltaTime < 70){
            incrementHashMap(Categories.MS_51_70);
        } else if (deltaTime < 90){
            incrementHashMap(Categories.MS_71_90);
        } else if (deltaTime < 120) {
            incrementHashMap(Categories.MS_91_120);
        } else {
            incrementHashMap(Categories.MS_120PLUS);
        }

        lastTime = FTCUtilities.getCurrentTimeMillis();
    }

    private void incrementHashMap(Categories cat){
        int c = timeCategorizations.get(cat);
        c++;
        timeCategorizations.put(cat, c);
    }

    public double getAverageDeltaTime(){
        return runningAverage;
    }

    public long getMaxDeltaTime(){
        return maxDeltaTime;
    }

    /**
     * Calculates what share of the total data points make up each categorization.
     * Does not save nickels, rather recalculates every time, so save or use sparingly if performance is of critical importance.
     * @return A key value set of categorizations that map to double percentages between 0 and 1
     */
    public HashMap<Categories, Double> getTimePercentages(){
        HashMap<Categories, Double> timePercentages = new HashMap<>();

        for (Categories c : timeCategorizations.keySet()) {
            double percent = (double)timeCategorizations.get(c) / (double)numberOfDataPoints; //no integer divisions
            timePercentages.put(c, percent);
        }
        return timePercentages;
    }

}
