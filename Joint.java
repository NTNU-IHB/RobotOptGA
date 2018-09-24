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
public class Joint {
    
    private double length;
    private double torsionalrigidity; //Nm/rad
    private double tiltingrigidity; //Nm/rad
    private double maxTorsionalTorque; //Nm
    private double maxTiltingTorque; //Nm
    private double weight; //kg
    
    public Joint (double leng, double stiffnessTor, double stiffnessTilt ){
        length = leng;
        torsionalrigidity = stiffnessTor;
        tiltingrigidity = stiffnessTilt;
    }
    
    public Joint (String in){
        if(in.equals("010")){
            torsionalrigidity = nmarcminToNm(42);
            tiltingrigidity = nmarcminToNm(225);
            maxTorsionalTorque = 180;
            maxTiltingTorque = 400;
            weight = 6.5;
        }else if (in.equals("025")){
            torsionalrigidity = nmarcminToNm(95);
            tiltingrigidity = nmarcminToNm(550);
            maxTorsionalTorque = 260;
            maxTiltingTorque = 550;
            weight = 10.0;
        }else if (in.equals("050")){
            torsionalrigidity = nmarcminToNm(205);
            tiltingrigidity = nmarcminToNm(560);
            maxTorsionalTorque = 675;
            maxTiltingTorque = 1355;
            weight = 21.8;
        }else if(in.equals("110")){
            torsionalrigidity = nmarcminToNm(650);
            tiltingrigidity = nmarcminToNm(1452);
            maxTorsionalTorque = 1750;
            maxTiltingTorque = 3280;
            weight = 45.5;
        }else if(in.equals("PRT")){
            torsionalrigidity = nmarcminToNm(300);
            tiltingrigidity = 1100*10*180/Math.PI;
            maxTorsionalTorque = 1000;
            maxTiltingTorque = 2000;
            weight = 12.0;
        }else if(in.equals("PRTlight")){
            torsionalrigidity = nmarcminToNm(100);
            tiltingrigidity = 500*10*180/Math.PI;
            maxTorsionalTorque = 200;
            maxTiltingTorque = 800;
            weight = 1.5;
        }
        
        
        else{
            System.out.println("Something wrong when initializing joint");
        }
            
    }

    
    private double nmarcminToNm(double nmarcmin){
        return nmarcmin*60*180/Math.PI;
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
     * @return the torsionalrigidity
     */
    public double getTorsionalrigidity() {
        return torsionalrigidity;
    }

    /**
     * @return the tiltingrigidity
     */
    public double getTiltingrigidity() {
        return tiltingrigidity;
    }

    /**
     * @return the maxTorsionalTorque
     */
    public double getMaxTorsionalTorque() {
        return maxTorsionalTorque;
    }

    /**
     * @return the maxTiltingTorque
     */
    public double getMaxTiltingTorque() {
        return maxTiltingTorque;
    }

    /**
     * @return the weight
     */
    public double getWeight() {
        return weight;
    }

    /**
     * @param weight the weight to set
     */
    public void setWeight(double weight) {
        this.weight = weight;
    }

   
   
    
    
}
