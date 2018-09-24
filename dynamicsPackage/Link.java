/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package dynamicsPackage;

/**
 *
 * @author Emil
 */
public class Link {

    private double length;
    private double pipelength;
    private double endplatelength;
    private double endplateweight;
    private double pipeweight;
    private double totalweight;
    private double diameter;
    private double wallThickness;
    private double stiffness;
    private double emod = 203;
    private double secMomentArea;
    private double maxTiltMoment;
    private double maxTorsMoment;
    private double tiltingStiffness; //Nm/m
    private double torsionalStiffness;

    public Link(double totleng, double diam, double wallthick, double endplatelen, double endplatewe) {
        length = totleng;
        pipelength = totleng-2*endplatelen;
        if(pipelength < 0.001){
            pipelength = 0.001;
        }
        diameter = diam;
        wallThickness = wallthick;
        endplatelength = endplatelen;
        endplateweight = endplatewe;
        calculatePipeWeight();
        totalweight = pipeweight+2*endplateweight;
        calculateTiltingStiffness();
        calculateTorsionalStiffness();
        calculateMaxTiltMoment();
        calculateMaxTorsMoment();
    }
    
    public void calculatePipeWeight(){
        double rod = diameter/2;
        double rid = rod-wallThickness;
        
        double area = ((rod*rod)-(rid*rid))*Math.PI;
        double volume = area*pipelength;
        
        pipeweight = volume * 7800;
        
    }

    public void calculateTiltingStiffness() {
        double rod = getDiameter() / 2;
        double rid = rod - getWallThickness();
        secMomentArea = Math.PI / 4 * ((rod * rod * rod * rod) - (rid * rid * rid * rid));
        stiffness = 2 * getEmod() * 1000000000 * secMomentArea / (pipelength *  pipelength);
        tiltingStiffness = stiffness;
    }
    
    public void calculateTorsionalStiffness(){
        double rod = getDiameter() / 2;
        double rid = rod - getWallThickness();
        secMomentArea = Math.PI / 2 * ((rod * rod * rod * rod) - (rid * rid * rid * rid));
        torsionalStiffness = secMomentArea * 75 * 1000000000 /pipelength;
    }
    
    public void calculateMaxTiltMoment(){
        double r = (diameter/2)-wallThickness/2;
        double t = wallThickness;
        maxTiltMoment = 1.035*emod*1000000000*r*t*t;
        
    }
    
    public void calculateMaxTorsMoment(){
        double rod = getDiameter() / 2;
        double maxShear = 505 / 1.73;
        double temp = maxShear * secMomentArea / rod;
    }
    
    public double getMaxTiltmoment(){
        return maxTiltMoment;
    }
    
    public double getMaxTorsMoment(){
        return maxTorsMoment;
    }
    
    public double getTiltingStiffness(){
        return tiltingStiffness;
    }
    
    public double getTorsionalStiffness(){
        return torsionalStiffness;
    }

    /**
     * @return the length
     */
    public double getLength() {
        return length;
    }

    /**
     * @param length the length to set
     */
    public void setLength(double length) {
        this.length = length;
    }

    /**
     * @return the diameter
     */
    public double getDiameter() {
        return diameter;
    }

    /**
     * @param diameter the diameter to set
     */
    public void setDiameter(double diameter) {
        this.diameter = diameter;
    }

    /**
     * @return the wallThickness
     */
    public double getWallThickness() {
        return wallThickness;
    }

    /**
     * @param wallThickness the wallThickness to set
     */
    public void setWallThickness(double wallThickness) {
        this.wallThickness = wallThickness;
    }

    /**
     * @return the emod
     */
    public double getEmod() {
        return emod;
    }

    /**
     * @param emod the emod to set
     */
    public void setEmod(double emod) {
        this.emod = emod;
    }
    public double getPipeLength(){
        return pipelength;
    }

    /**
     * @return the totalweight
     */
    public double getTotalweight() {
        return totalweight;
    }

}
