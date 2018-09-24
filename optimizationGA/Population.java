/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package optimizationGA;

import dynamicsPackage.RobotManipulator2;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import kinematics.Kinematics;

/**
 *
 * @author Emil
 */
public class Population {

    private Individual[] individuals;
    private int popSize;

    public Population(int populationSize, boolean initialize) {
        individuals = new Individual[populationSize];
        popSize = populationSize;

        if (initialize) {
            for (int i = 0; i < populationSize; i++) {
                if (i == 13) {
                    individuals[i] = new Individual();
                } else {
                    individuals[i] = new Individual();
                }
            }
        }
    }

    /**
     * @return the popSize
     */
    public int getPopSize() {
        return popSize;
    }

    public Individual getIndividual(int index) {
        return individuals[index];
    }

    /**
     * @return the individuals
     */
    public Individual[] getIndividuals() {
        return individuals;
    }

    public void setIndividual(Individual in, int index) {
        individuals[index] = in;
    }

    /**
     * Sorts the population, with the fittest first, and then descending
     */
    public void sort() {

        List<Individual> templist = Arrays.asList(individuals);
        Collections.sort(templist, new CustomComparator().reversed());

        for (int j = 0; j < templist.size(); j++) {
            individuals[j] = templist.get(j);
        }
        for (int i = 0; i < 4; i++) {
            RobotManipulator2 testman = new RobotManipulator2(individuals[i].getUnnormalizedChromosome());
            int temp = testman.getLimitingJoint();
            System.out.println("Best individual nr " + i + ". Score: " + individuals[i].getFitness() + ". Limiting joint: " + temp);
            double payload = testman.getPayload();

            double reach = testman.getReach();
            System.out.println("Payload: " + payload + ". Reach: "+reach+". Weight: "+testman.getWeight()+". Sitffness XYZ: "
                    +testman.getStiffnessXdir()+", "+testman.getStiffnessYdir()+", "+testman.getStiffnessZdir());

        }
        System.out.println("Worst individual: " + (popSize - 1) + ". Score: " + individuals[popSize - 1].getFitness());

    }

    public void newRandomIndividual(int index) {
        individuals[index] = new Individual();
    }
    
    

    public void printIndividuals() {
        for (int i = 0; i < individuals.length; i++) {
            System.out.println("Ind " + i + " with fitness: " + individuals[i].getFitness() + "");
            String temp = Kinematics.vectorToString(individuals[i].getUnnormalizedChromosome());
            System.out.println(temp);
        }
    }

}
