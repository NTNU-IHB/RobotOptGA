/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package optimizationGA;

import dynamicsPackage.RobotManipulator2;
import kinematics.Kinematics;

/**
 *
 * @author Emil
 */
public class Individual {

    private int numGenes = 12;
    private double[] chromosome = new double[getNumGenes()];
    private double fitness = 1000;
    private double[] randomScalarMultiplyer = {1.5, 1.5, 1.5, 1.5, 0.24, 0.24, 0.24, 0.24, 0.009, 0.009, 0.009, 0.009};
    private double[] randomScalarOffset = {0.2, 0.2, 0.2, 0.2, 0.01, 0.01, 0.01, 0.01, 0.0001, 0.0001, 0.0001, 0.0001};
    

    public Individual() {
        initializeRandomIndividual();
    }

    public Individual(double[] normalizedGenes) {
        if (normalizedGenes.length == numGenes) {
            for (int i = 0; i < normalizedGenes.length; i++) {
                chromosome[i] = normalizedGenes[i];
                 
            }
        } else {
            System.out.println("Wrong number of genes");
        }
    }
    
    public Individual(double[] unNormalized, boolean notNormalized) {
        if (unNormalized.length == numGenes) {
            for (int i = 0; i < unNormalized.length; i++) {
               
                chromosome[i] = ((unNormalized[i] - randomScalarOffset[i]) / randomScalarMultiplyer[i]) ;
                
            }
        } else {
            System.out.println("Wrong number of genes");
        }
    }
    

    public void initializeRandomIndividual() {

        for (int i = 0; i < getNumGenes(); i++) {
            getNormalizedChromosome()[i] = Math.random();
        }

    }

    public void updateFitnessIfLower(double fit) {
        if (fit < getFitness()) {
            fitness = fit;
        }
    }

    public void addToFitness(double fitbit) {
        fitness = fitness + fitbit;
    }

    public void subtractFromFitness(double notfit) {
        fitness = fitness - notfit;
    }

    public void setFitness(double fitt) {
        fitness = fitt;
    }

    /**
     * @return the chromosome
     */
    public double[] getNormalizedChromosome() {
        return chromosome;
    }

    public double[] getUnnormalizedChromosome() {
        double[] temp = new double[chromosome.length];
        for (int i = 0; i < temp.length; i++) {
            temp[i] = (chromosome[i] * randomScalarMultiplyer[i]) + randomScalarOffset[i];
        }
        return temp;
    }

    /**
     * @param chromosome the chromosome to set
     */
    public void setNormalizedChromosome(double[] chromosome) {
        this.chromosome = chromosome;
    }

    /**
     * @return the fitness
     */
    public double getFitness() {
        return fitness;
    }

    public void setNormalizedGene(double in, int i) {
        chromosome[i] = in;
    }

    /**
     * @return the numGenes
     */
    public int getNumGenes() {
        return numGenes;
    }
    
    

    /**
     * @return the randomScalarMultiplyer
     */
    public double[] getRandomScalarMultiplyer() {
        return randomScalarMultiplyer;
    }

    /**
     * @return the randomScalarOffset
     */
    public double[] getRandomScalarOffset() {
        return randomScalarOffset;
    }

    public String toRobotPropertiesString() {
        RobotManipulator2 testman = new RobotManipulator2(this.getUnnormalizedChromosome());
        int temp = testman.getLimitingJoint();
        String part1 = ("Best individual. Score: " + this.getFitness() + ". Limiting joint: " + temp);
        double payload = testman.getPayload();
        double reach = testman.getReach();
        String part2 = (" Payload: " + payload + ". Reach: " + reach + ". Weight: " + testman.getWeight() + ". Sitffness XYZ: "
                + testman.getStiffnessXdir() + "  " + testman.getStiffnessYdir() + "  " + testman.getStiffnessZdir());
        return part1+part2;
    }

}
