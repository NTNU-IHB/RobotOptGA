/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package optimizationGA;

/**
 *
 * @author Emil
 */
public class Evolution {

    private int numElite = 3;
    private double mutRate = 0.5;

    private int nearSearchPrElite = 0; // 4

    public Evolution(boolean extraExploit) {
        if (extraExploit){
            nearSearchPrElite = 4;
        }
    }

    public Population evolvePopOnce(Population inpop) {
        Population pop = new Population(inpop.getPopSize(), false);

        int numInvalid = checkNumInvalid(inpop);
        Population popWithoutUnvalid = inpop;

        int eliteSaved = 0;
        for (int i = 0; i < numElite; i++) {
            if (numInvalid < ((pop.getPopSize()) - i)) {
                pop.setIndividual(inpop.getIndividual(i), i);
                eliteSaved++;
            }
        }

        int count = eliteSaved;

        int eliteGradientDescent = eliteSaved * nearSearchPrElite;

        for (int k = 0; k < eliteSaved; k++) {
            Individual tempIn = pop.getIndividual(k);
            for (int l = 0; l < nearSearchPrElite; l++) {
                double[] newchrom = new double[12];
                for (int j = 0; j < tempIn.getNumGenes(); j++) {
                    double gain = 0.1;
                    double tempgene = (tempIn.getNormalizedChromosome()[j]) + (Math.random() - 0.5) * gain;
                    if (tempgene < 0.0) {
                        tempgene = 0.0;
                    } else if (tempgene > 1.0) {
                        tempgene = 1.0;
                    }
                    newchrom[j] = tempgene;
                }

                Individual elitegrad = new Individual(newchrom);
                pop.setIndividual(elitegrad, count);
                count++;
            }
        }

        for (int j = (eliteSaved + eliteGradientDescent); j < pop.getPopSize(); j++) {
            Individual par1 = popWithoutUnvalid.getIndividual(j);
            Individual par2 = popWithoutUnvalid.getIndividual(pop.getPopSize() - 1 - j);
            Individual child = crossover(par1, par2);

            Individual mutatedChild = mutate(child);

            pop.setIndividual(mutatedChild, j);

        }

        return pop;
    }

    public int checkNumInvalid(Population pop) {
        int counter = 0;

        while ((pop.getIndividual(pop.getPopSize() - 1 - counter).getFitness() <= 0.0)) {

            
            counter++;
            if (counter == pop.getPopSize()) {
                break;
            }
        }
        return counter;

    }

    public Population dropUnvalidCandidates(Population pop) {
        int counter = 0;
        while ((pop.getIndividual(pop.getPopSize() - 1 - counter).getFitness() <= 0.0)) {

            pop.newRandomIndividual(pop.getPopSize() - 1 - counter);

            counter++;
            if (counter == pop.getPopSize()) {
                break;
            }
        }

        System.out.println("Dropped " + counter + " unvalid solutions.");
        Population outpop = pop;
        return outpop;

    }

    public Individual crossover(Individual par1, Individual par2) {
        Individual child = new Individual();
        for (int i = 0; i < child.getNumGenes(); i++) {
            if (Math.random() < 0.5) {
                child.setNormalizedGene(par1.getNormalizedChromosome()[i], i);
            } else {
                child.setNormalizedGene(par2.getNormalizedChromosome()[i], i);
            }
        }

        return child;
    }

    public Individual mutate(Individual in) {

        for (int i = 0; i < in.getNormalizedChromosome().length; i++) {
            if (Math.random() <= mutRate) {
                double gain = 0.3;
                double tempgene = (in.getNormalizedChromosome()[i]) + (Math.random() - 0.5) * gain;
                //double gene = Math.random() * 1.5;

                if (tempgene < 0.0) {
                    tempgene = 0.0;
                } else if (tempgene > 1.0) {
                    tempgene = 1.0;
                }

                in.setNormalizedGene(tempgene, i);
            }
        }
        return in;
    }

}
