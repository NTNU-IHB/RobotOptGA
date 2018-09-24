/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package optimizationGA;

import java.util.Comparator;

/**
 *
 * @author Emil
 */
public class CustomComparator implements Comparator<Individual> {
    @Override
    public int compare(Individual o1, Individual o2) {
        Double temp1 = o1.getFitness();
        Double temp2 = o2.getFitness();
        return temp1.compareTo(temp2);
    }
}