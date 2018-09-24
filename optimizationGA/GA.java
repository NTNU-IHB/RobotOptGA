/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package optimizationGA;

import dynamicsPackage.RobotManipulator2;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Iterator;
import kinematics.Kinematics;

/**
 *
 * @author Emil
 */
public class GA {

    public static void main(String[] args) {

        long startMillis = System.currentTimeMillis();
        boolean extraExploit = true;
        boolean optReach = true;
        
        int numberOfRuns = 20;
        int numberOfIterations = 200;
        int populationSize = 50;

        double[][] allFitness = new double[numberOfRuns][numberOfIterations+1];
        double[][] allPayload = new double[numberOfRuns][numberOfIterations+1];
        double[][] allReach = new double[numberOfRuns][numberOfIterations+1];
        double[][] allStiffness = new double[numberOfRuns][numberOfIterations+1];
        double[][] allWeight = new double[numberOfRuns][numberOfIterations+1];

        Individual[] tempBest = new Individual[numberOfRuns];

        for (int mult = 0; mult < numberOfRuns; mult++) {
            System.out.println("Run number: "+mult);
            Population pop = new Population(populationSize, true);
            FitnessEvaluation eval = new FitnessEvaluation(optReach);
            Evolution evolution = new Evolution(extraExploit);

            double[] fitnesses = new double[numberOfIterations+1];
            double[] payloadList = new double[numberOfIterations+1];
            double[] reachList = new double[numberOfIterations+1];
            double[] stiffnessList = new double[numberOfIterations+1];
            double[] weightList = new double[numberOfIterations+1];

            eval.updateCostOfPop(pop);
            pop.sort();
            pop.printIndividuals();
            Individual bestIn = pop.getIndividual(0);
            RobotManipulator2 testman = new RobotManipulator2(bestIn.getUnnormalizedChromosome());
            double bestFitness1 = bestIn.getFitness();
            fitnesses[0] = bestFitness1;

            double payload1 = testman.getPayload();
            payloadList[0] = payload1;

            double reach1 = testman.getReach();
            reachList[0] = reach1;

            double stiffness1 = testman.getStiffnessAVG();
            stiffnessList[0] = stiffness1;

            double weight1 = testman.getWeight();
            weightList[0] = weight1;

            for (int i = 0; i < numberOfIterations; i++) {
                System.out.println("Generation number: "+i);
                Population temppop = evolution.evolvePopOnce(pop);
                eval.updateCostOfPop(temppop);

                temppop.sort();
                temppop.printIndividuals();
                Individual bestInd = temppop.getIndividual(0);
                RobotManipulator2 testman2 = new RobotManipulator2(bestInd.getUnnormalizedChromosome());
                double bestFitness = bestInd.getFitness();
                fitnesses[i + 1] = bestFitness;

                double payload = testman2.getPayload();
                payloadList[i + 1] = payload;

                double reach = testman2.getReach();
                reachList[i + 1] = reach;

                double stiffness = testman2.getStiffnessAVG();
                stiffnessList[i + 1] = stiffness;

                double weight = testman2.getWeight();
                weightList[i + 1] = weight;
                pop = temppop;

            }
            
            tempBest[mult] = pop.getIndividual(0);

            allFitness[mult] = fitnesses;
            allPayload[mult] = payloadList;
            allReach[mult] = reachList;
            allStiffness[mult] = stiffnessList;
            allWeight[mult] = weightList;

        }

        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy_MM_dd_HH_mm_ss");
        LocalDateTime lul = LocalDateTime.now();
        String timestamp = lul.format(formatter);
        String extraEX;
        String reachOrPayload;
        if (extraExploit) {
            extraEX = "exploit";
        } else {
            extraEX = "standard";
        }
        if (optReach) {
            reachOrPayload = "reach";
        } else {
            reachOrPayload = "payload";
        }
        

        FileWriter fwError = null;
        try {

            fwError = new FileWriter("C:\\Users\\Emil\\Dropbox\\PhD\\RobotOpt paper\\results\\multiple\\" + "result " + timestamp + " " + extraEX +" "+reachOrPayload+ ".csv");
        } catch (IOException e) {
            e.printStackTrace();
        }
        PrintWriter pw = new PrintWriter(fwError);

        pw.print("Fitness, ");
        pw.print("Payload, ");
        pw.print("Reach, ");
        pw.print("Stiffness, ");
        pw.println("Weight");

       
        StringBuilder sb = new StringBuilder();
        int count = 0;
        while (count < numberOfIterations+1) {
            double fitAvg = 0;
            double payAvg = 0;
            double reachAvg = 0;
            double stiffAvg = 0;
            double weightAvg = 0;
            for (int inc = 0; inc < numberOfRuns; inc++) {
                fitAvg = fitAvg + allFitness[inc][count] / numberOfRuns;
                payAvg = payAvg + allPayload[inc][count] / numberOfRuns;
                reachAvg = reachAvg + allReach[inc][count] / numberOfRuns;
                stiffAvg = stiffAvg + allStiffness[inc][count] / numberOfRuns;
                weightAvg = weightAvg + allWeight[inc][count] / numberOfRuns;
                pw.print(allFitness[inc][count] + ",");
                pw.print(allPayload[inc][count] + ",");
                pw.print(allReach[inc][count] + ",");
                pw.print(allStiffness[inc][count] + ",");
                pw.print(allWeight[inc][count]+ ",");
                pw.print(" ,");
            }
            System.out.print(fitAvg + ", ");
            pw.print(fitAvg + ",");
            pw.print(payAvg + ",");
            pw.print(reachAvg + ",");
            pw.print(stiffAvg + ",");
            pw.println(weightAvg);
            count++;

        }
        pw.println(" ");
        
        Individual bessst = tempBest[0];
        for (Individual ind : tempBest){
            if(ind != null){
            double newfit = ind.getFitness();
            double oldfit = bessst.getFitness();
            if(newfit > oldfit){
                bessst = ind;
            }
            }
        }
        
        
        pw.println(bessst.toRobotPropertiesString());
        double[] unNormChromes = bessst.getUnnormalizedChromosome();
        for (int i = 0; i < unNormChromes.length; i++) {
            pw.print(unNormChromes[i] + ",");
        }

        pw.close();

        long endMillis = System.currentTimeMillis();
        long timeElapsed = endMillis - startMillis;
        long inSeconds = timeElapsed / 1000;
        System.out.println(" ");
        System.out.println("Time elapsed in seconds: " + inSeconds);

    }

}
