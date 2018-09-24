/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package optimizationGA;

//import dynamicsPackage.RobotManipulator;
import dynamicsPackage.RobotManipulator2;
import kinematics.Kinematics;

/**
 *
 * @author Emil
 */
public class FitnessEvaluation {

  
    private double[] worstcasejoints = {0.0, Math.PI / 2, 0.0, -Math.PI / 2, 0.0, 0.0};

    double[] temppjoint = {Math.random() * 1.3, Math.random() * 1.3, Math.random() * 1.3, Math.random() * 1.3, Math.random() * 1.3, Math.random() * 1.3};

    
    private boolean optForReach;

    public FitnessEvaluation(boolean optReach) {
        optForReach = optReach;

    }

    public void calculateFitnessForIndividual(Individual in) {
        double payloadRequirement;
        boolean optimizeForMaxPayload;
        double reachRequirement;
        boolean optimizeForMaxReach;
        double minStiffness;
        if (optForReach) {
            payloadRequirement = 100.0;
            optimizeForMaxPayload = false;
            reachRequirement = 2.0;
            optimizeForMaxReach = true;
            minStiffness = 4000.0;
        } else {
            payloadRequirement = 100.0;
            optimizeForMaxPayload = true;
            reachRequirement = 1.0;
            optimizeForMaxReach = false;
            minStiffness = 4000.0;
        }
        double[] chromosome = in.getUnnormalizedChromosome();
        RobotManipulator2 testman = new RobotManipulator2(chromosome);

        double payloadFitness = calculateFitnessPayload(in, testman, payloadRequirement, optimizeForMaxPayload);
        
        in.setFitness(payloadFitness);

        double xyzStiffnessFit = calculateFitnessXYZstiffness(in, testman, minStiffness);
        in.addToFitness(xyzStiffnessFit);

        
        double reachFitness = calculateReachFitness(in, testman, reachRequirement, optimizeForMaxReach);
        
        in.addToFitness(reachFitness);

        double weight = calculateWeightFitness(in, testman);

        in.subtractFromFitness(weight);

        
    }

    public double calculateWeightFitness(Individual in, RobotManipulator2 inMan) {
        RobotManipulator2 testman = inMan;
        double weight = testman.getWeight();
        return weight;

    }

    public double calculateFitnessXYZstiffness(Individual in, RobotManipulator2 inMan, double minStiffness) {
        RobotManipulator2 testman = inMan;
        double xstif = testman.getStiffnessXdir();
        double ystif = testman.getStiffnessYdir();
        double zstif = testman.getStiffnessZdir();
        double sumStif = (xstif + ystif + zstif);
        double result = sumStif/3 * 0.001;
        if (sumStif < minStiffness) {
            result = -1000.0;
        }
        return result;
    }

    public double calculateReachFitness(Individual in, RobotManipulator2 inMan, double reachRequirement, boolean optimizeForMaxReach) {
        RobotManipulator2 testman = inMan;
        double reach = testman.getReach();
        if (reach > reachRequirement) {
            if (!optimizeForMaxReach) {
                reach = reachRequirement;
            }
            reach = reach * 100;
        } else {
            reach = -1000.0;
        }
        return reach;
    }

    public double calculateFitnessPayload(Individual in, RobotManipulator2 inMan, double payloadRequirement, boolean optimizeForMaxPayload) {
        RobotManipulator2 testman = inMan;
        double payload = testman.getPayload();
       
        if (payload > payloadRequirement) {
            if (!optimizeForMaxPayload) {
                payload = payloadRequirement;
            }
        } else {
            payload = -100000.0;
        }
        return payload;
    }
    
    public void updateCostOfPop(Population pop) {
        for (int i = 0; i < pop.getPopSize(); i++) {
            calculateFitnessForIndividual(pop.getIndividual(i));
        }
    }


    /**
     * @return the worstcasejoints
     */
    public double[] getWorstcasejoints() {
        return worstcasejoints;
    }

    /**
     * @param worstcasejoints the worstcasejoints to set
     */
    public void setWorstcasejoints(double[] worstcasejoints) {
        this.worstcasejoints = worstcasejoints;
    }

}
