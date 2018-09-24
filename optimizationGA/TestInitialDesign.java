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
public class TestInitialDesign {
    
    public static void main (String[] args){
        System.out.println("hi");
        
        double[] initDesignGenes = new double[] {1.34, 1.34, 0.5, 0.5, 0.15, 0.15, 0.05, 0.05, 0.001, 0.001, 0.001, 0.001};
        Individual in = new Individual(initDesignGenes, true);
        System.out.println((in.toRobotPropertiesString()));
    }
    
}
