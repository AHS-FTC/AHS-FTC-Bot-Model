package edu.ahs.robotics;

public class GearRatio {
    private int inputTeeth;
    private int outputTeeth;

    public GearRatio (int inputTeeth, int outputTeeth){
        this.inputTeeth = inputTeeth;
        this.outputTeeth = outputTeeth;
    }

    public GearRatio () {
        inputTeeth = 1;
        outputTeeth = 1;
    }

    public double getRatioAsDouble(){
        double ratio = (double)inputTeeth/(double)outputTeeth;
        return ratio;
    }
    public int getInputTeeth () {
        return inputTeeth;
    }

    public int getOutputTeeth () {
        return outputTeeth;
    }

//    public void flip() {
//        int inputTeeth2 = inputTeeth;
//        int outputTeeth2 = outputTeeth;
//        inputTeeth = outputTeeth2;
//        outputTeeth = inputTeeth2;
//    }
}
