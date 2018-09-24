/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package dynamicsPackage;

import kinematics.Kinematics;

/**
 *
 * @author Emil
 */
public class RobotManipulator2 {

    private Link[] links;
    private Joint[] joints;

    private final double[] jointValues = {0.0, Math.PI / 2, 0.0, -Math.PI / 2, 0.0, 0.0};
    private Kinematics kin;

    private double[][][] maxForcesDueToJoint;
    private double[][][] maxForcesDueToLink;

    private final int numJoints = 6;

    private double manipulatorWeight;
    private double[][] maxPayloadDueToLink = new double[6][2];
    private double[][] maxPayloadDueToJoint = new double[6][2];

    private double payload;
    private double reach;
    private double[] stiffnessesXYZ;
    private int limitingJoint;

    public RobotManipulator2() {
        links = new Link[6];
        joints = new Joint[6];

        maxForcesDueToJoint = new double[6][3][2];
        maxForcesDueToLink = new double[6][3][2];
        for (int k = 0; k < 6; k++) {
            for (int l = 0; l < 2; l++) {
                for (int j = 0; j < 3; j++) {
                    maxForcesDueToJoint[k][j][l] = 100000.0;
                    maxForcesDueToLink[k][j][l] = 1000000.0;
                }
            }

        }
        for (int i = 0; i < numJoints; i++) {

            links[i] = new Link(1.000, 0.250, 0.002, 0.200, 2);

        }
        double[] initDesignGenes = new double[]{1.34, 1.34, 0.5, 0.5, 0.15, 0.15, 0.05, 0.05, 0.001, 0.001, 0.001, 0.001};
        links[0] = new Link(0.2, 0.2, 0.005, 0.002, 3);
        links[1] = new Link(initDesignGenes[0], initDesignGenes[4], initDesignGenes[8], 0.05, 2);
        links[2] = new Link(initDesignGenes[1], initDesignGenes[5], initDesignGenes[9], 0.05, 2);
        links[3] = new Link(0.2, 0.2, 0.005, 0.002, 2);
        links[4] = new Link(initDesignGenes[2], initDesignGenes[6], initDesignGenes[10], 0.05, 0.5);
        links[5] = new Link(initDesignGenes[3], initDesignGenes[7], initDesignGenes[11], 0.05, 0.5);
        double[] adjustableLinksLengths = {links[1].getLength(), links[2].getLength(), links[4].getLength(), links[5].getLength()};
        kin = new Kinematics(adjustableLinksLengths);

        joints[0] = new Joint("PRT");
        joints[1] = new Joint("110");
        joints[2] = new Joint("050");
        joints[3] = new Joint("025");
        joints[4] = new Joint("010");
        joints[5] = new Joint("PRTlight");

        calculateWeight();
        calculatePayload();
        calculateReach();
        calculateStiffnessXYZ();
    }

    public RobotManipulator2(double[] in) {
        links = new Link[6];
        joints = new Joint[6];

        maxForcesDueToJoint = new double[6][3][2];
        maxForcesDueToLink = new double[6][3][2];
        for (int k = 0; k < 6; k++) {
            for (int l = 0; l < 2; l++) {
                for (int j = 0; j < 3; j++) {
                    maxForcesDueToJoint[k][j][l] = 100000.0;
                    maxForcesDueToLink[k][j][l] = 1000000.0;
                }
            }

        }
        for (int i = 0; i < numJoints; i++) {

            links[i] = new Link(1.000, 0.250, 0.002, 0.200, 2);

        }

        links[0] = new Link(0.2, 0.2, 0.005, 0.002, 3);
        links[1] = new Link(in[0], in[4], in[8], 0.05, 2);
        links[2] = new Link(in[1], in[5], in[9], 0.05, 2);
        links[3] = new Link(0.2, 0.2, 0.005, 0.002, 2);
        links[4] = new Link(in[2], in[6], in[10], 0.05, 0.5);
        links[5] = new Link(in[3], in[7], in[11], 0.05, 0.5);
        double[] adjustableLinksLengths = {links[1].getLength(), links[2].getLength(), links[4].getLength(), links[5].getLength()};
        kin = new Kinematics(adjustableLinksLengths);

        joints[0] = new Joint("PRT");
        joints[1] = new Joint("110");
        joints[2] = new Joint("050");
        joints[3] = new Joint("025");
        joints[4] = new Joint("010");
        joints[5] = new Joint("PRTlight");

        calculateWeight();
        calculatePayload();
        calculateReach();
        calculateStiffnessXYZ();
    }

    private void calculateReach() {
        double[] vec = getJointCenterPos()[6];
        double temp = kin.calcVectorLength(vec);
        reach = temp;
    }

    public void calculateStiffnessXYZ() {
        double[] tempstiffness = new double[3];
        for (int j = 0; j < 3; j++) {
            double[] xjointtemp = calcJointStiffnessXYZ()[j];
            double[] xlinktemp = calcLinkStiffnessXYZ()[j];
            double inverseStiff = 0;

            for (int i = 0; i < 6; i++) {
                inverseStiff = inverseStiff + (1 / xjointtemp[i]) + (1 / xlinktemp[i]);
            }
            tempstiffness[j] = 1 / inverseStiff;

        }
        stiffnessesXYZ = tempstiffness;
    }

    public double[][] calcJointStiffnessXYZ() {
        double[][] jointStiffnessXYZ = new double[3][6];
        for (int xyz = 0; xyz < 3; xyz++) {
            for (int i = 0; i < 6; i++) {
                double[] Fvector = {1, 0, 0};
                switch (xyz) {
                    case 0:
                        Fvector = new double[]{1, 0, 0};
                        break;
                    case 1:
                        Fvector = new double[]{0, 1, 0};
                        break;
                    case 2:
                        Fvector = new double[]{0, 0, 1};
                        break;
                    default:
                        break;
                }
                double[] Px6vec = getPx6()[i];
                double[] momentvec = kin.crossVectors(Px6vec, Fvector);
                double[] tempZaxis = getZvectorsForJoint()[i];
                double tiltarm = Math.abs(kin.crossVectorsLength(momentvec, tempZaxis));
                double torsarm = Math.abs(kin.dotVectorLength(momentvec, getZvectorsForJoint()[i]));
                double jointTiltStiffness = joints[i].getTiltingrigidity() / (tiltarm * tiltarm);
                double jointTorsStiffness = joints[i].getTorsionalrigidity() / (torsarm * torsarm);
                double temp = (1 / jointTiltStiffness) + (1 / jointTorsStiffness);
                double tempjointStiffness = 1 / temp;

                jointStiffnessXYZ[xyz][i] = tempjointStiffness;
            }
        }
        return jointStiffnessXYZ;
    }

    public double[][] calcLinkStiffnessXYZ() {
        double[][] linkStiffnessXYZ = new double[3][6];
        for (int xyz = 0; xyz < 3; xyz++) {
            for (int i = 0; i < 6; i++) {
                double[] Fvector = {1, 0, 0};
                switch (xyz) {
                    case 0:
                        Fvector = new double[]{1, 0, 0};
                        break;
                    case 1:
                        Fvector = new double[]{0, 1, 0};
                        break;
                    case 2:
                        Fvector = new double[]{0, 0, 1};
                        break;
                    default:
                        break;
                }
                double[] Px6vec = getPx6()[i];
                double[] momentvec = kin.crossVectors(Px6vec, Fvector);
                double[] tempZaxis = getZvectorsForLinks()[i];
                double tiltarm = Math.abs(kin.crossVectorsLength(momentvec, tempZaxis));
                double torsarm = Math.abs(kin.dotVectorLength(momentvec, tempZaxis));
                double linkTiltStiffness = links[i].getTiltingStiffness() / (tiltarm * tiltarm);
                double linkTorsStiffness = links[i].getTorsionalStiffness() / (torsarm * torsarm);
                double temp = (1 / linkTiltStiffness) + (1 / linkTorsStiffness);
                double templinkStiffness = 1 / temp;

                linkStiffnessXYZ[xyz][i] = templinkStiffness;
            }
        }
        return linkStiffnessXYZ;
    }

    public double getStiffnessXdir() {
        return stiffnessesXYZ[0];
    }

    public double getStiffnessYdir() {
        return stiffnessesXYZ[1];
    }

    public double getStiffnessZdir() {
        return stiffnessesXYZ[2];
    }

    public double getStiffnessAVG() {
        double temp = (stiffnessesXYZ[0] + stiffnessesXYZ[1] + stiffnessesXYZ[2]) / 3;
        return temp;
    }

    private void calculatePayload() {
        double maxPayloadDueToLinks = calculateMaxPayloadDueToLinks();
        double maxPayloadDueToJoints = calculateMaxPayloadDueToJoints();

        double temp_payload = maxPayloadDueToJoints;
        if (maxPayloadDueToLinks < temp_payload) {
            temp_payload = maxPayloadDueToLinks;
        }
        payload = temp_payload;
    }

    private double[][] getPx6() {
        double[][] Px6 = kin.getPx6WorldFrame(jointValues);
        return Px6;
    }

    private double[][] getZvectorsForJoint() {
        double[][] zVectors = kin.getJointZVectorsFromJointValues(jointValues);
        return zVectors;
    }


    private double[][] getZvectorsForLinks() {
        double[][] zVectorsLinks = new double[6][3];
        for (int i = 0; i < 6; i++) {
            double[] temp = kin.vectorSubtraction(getJointCenterPos()[i + 1], getJointCenterPos()[i]);
            zVectorsLinks[i] = kin.normalizeVector(temp);
        }
        return zVectorsLinks;
    }

    private double[][] getJointCenterPos() {
        double[][] jointPos = kin.getJointCentersWorldFrame(jointValues);
        return jointPos;
    }

    private double calculateMaxPayloadDueToLinks() {

        for (int i = 0; i < 6; i++) {
            double[] Zvector = {0, 0, 1};
            double[] Px6vec = getPx6()[i];
            double[] momentvec = kin.crossVectors(Px6vec, Zvector);

            double tiltarm = kin.crossVectorsLength(momentvec, getZvectorsForLinks()[i]);
            double torsarm = kin.dotVectorLength(momentvec, getZvectorsForLinks()[i]);
            double maxTilt = links[i].getMaxTiltmoment() - calcGravTorqueAffectingLink(i)[0];
            double maxTors = links[i].getMaxTorsMoment() - calcGravTorqueAffectingLink(i)[1];
            double FzmaxTilt = maxTilt / tiltarm;
            double FzmaxTors = maxTors / torsarm;
            maxPayloadDueToLink[i][0] = FzmaxTilt;
            maxPayloadDueToLink[i][1] = FzmaxTors;
        }

        double tempMin = 1000000.0;
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 2; j++) {
                double tempVal = maxPayloadDueToLink[i][j];
                if (tempMin > tempVal) {
                    tempMin = tempVal;
                }
            }
        }
        return tempMin;
    }

    private double calculateMaxPayloadDueToJoints() {

        for (int i = 0; i < 6; i++) {
            double[] Zvector = {0, 0, 1};
            double[] Px6vec = getPx6()[i];
            double[] momentvec = kin.crossVectors(Px6vec, Zvector);

            double tiltarm = Math.abs(kin.crossVectorsLength(momentvec, getZvectorsForJoint()[i]));
            double torsarm = Math.abs(kin.dotVectorLength(momentvec, getZvectorsForJoint()[i]));
            double maxTiltTorque = joints[i].getMaxTiltingTorque() - calcGravTorqueAffectingJoint(i)[0];
            double maxTorsTorque = joints[i].getMaxTorsionalTorque() - calcGravTorqueAffectingJoint(i)[1];
            double FzmaxTilt = maxTiltTorque / tiltarm;
            double FzmaxTors = maxTorsTorque / torsarm;
            maxPayloadDueToJoint[i][0] = FzmaxTilt;
            maxPayloadDueToJoint[i][1] = FzmaxTors;
        }
        //find minimum
        double tempMin = 1000000.0;
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 2; j++) {
                double tempVal = maxPayloadDueToJoint[i][j];
                if (tempMin > tempVal) {
                    tempMin = tempVal;
                    limitingJoint = i;
                }
            }
        }
        return tempMin;
    }

    public double[] calcGravTorqueAffectingJoint(int jointnumber) {
        double[] out = new double[]{0, 0};
        //Link weight
        for (int i = jointnumber; i < (6); i++) {
            double weight = links[i].getTotalweight();
            double[] startpos = getJointCenterPos()[i];
            double[] endpos = getJointCenterPos()[i + 1];
            double[] cog = kin.vectorMidpoint(startpos, endpos);
            double[] temptorq = calcGravTorqueOnJointFromItem(weight, cog, jointnumber);
            out[0] = out[0] + temptorq[0];
            out[1] = out[1] + temptorq[1];
        }

        //Joint weight
        for (int i = jointnumber; i < (6); i++) {
            double weight = joints[i].getWeight();
            double[] startpos = getJointCenterPos()[i];

            double[] temptorq = calcGravTorqueOnJointFromItem(weight, startpos, jointnumber);
            out[0] = out[0] + temptorq[0];
            out[1] = out[1] + temptorq[1];
        }

        return out;
    }

    public double[] calcGravTorqueAffectingLink(int linknumber) {
        double[] out = {0, 0};
        //Link weights
        for (int i = linknumber; i < (6); i++) {
            double weight = links[i].getTotalweight();
            double[] startpos = getJointCenterPos()[i];
            double[] endpos = getJointCenterPos()[i + 1];
            double[] cog = kin.vectorMidpoint(startpos, endpos);
            double[] temptorq = calcGravTorqueOnLinkFromItem(weight, cog, linknumber);
            out[0] = out[0] + temptorq[0];
            out[1] = out[1] + temptorq[1];
        }

        //Joint weights
        for (int i = (linknumber + 1); i < (6); i++) {
            double weight = joints[i].getWeight();
            double[] startpos = getJointCenterPos()[i];

            double temptorq[] = calcGravTorqueOnLinkFromItem(weight, startpos, linknumber);
            out[0] = out[0] + temptorq[0];
            out[1] = out[1] + temptorq[1];
        }
        return out;

    }

    public double[] calcGravTorqueOnLinkFromItem(double weight, double[] cogWorldFrame, int linknumber) {
        double[] Zvector = {0, 0, 1};
        double[] vecLinkStartPointToCOG = kin.vectorSubtraction(cogWorldFrame, getJointCenterPos()[linknumber]);
        double[] momentVec = kin.crossVectors(vecLinkStartPointToCOG, getZvectorsForLinks()[linknumber]);

        double tiltarm = kin.crossVectorsLength(momentVec, Zvector);
        double torsarm = kin.dotVectorLength(momentVec, Zvector);

        double MgravTilt = weight * 9.81 * tiltarm;
        double MgravTors = weight * 9.81 * torsarm;
        double[] returnTorq = {MgravTilt, MgravTors};
        return returnTorq;

    }

    public double[] calcGravTorqueOnJointFromItem(double weight, double[] cogWorldFrame, int jointnumber) {

        double[] Zvector = {0, 0, 1};
        double[] vecJointToCOG = kin.vectorSubtraction(cogWorldFrame, getJointCenterPos()[jointnumber]);
        double[] momentvec = kin.crossVectors(vecJointToCOG, getZvectorsForJoint()[jointnumber]);

        double tiltarm = (kin.crossVectorsLength(momentvec, Zvector));
        double torsarm = (kin.dotVectorLength(momentvec, Zvector));

        double MgravTilt = weight * 9.81 * tiltarm;
        double MgravTors = weight * 9.81 * torsarm;

        double[] returnTorq = {MgravTilt, MgravTors};
        return returnTorq;

    }

    public double getWeight() {
        return manipulatorWeight;
    }

    private void calculateWeight() {
        double weight = 0;
        for (int i = 0; i < 6; i++) {
            double temp1 = joints[i].getWeight();
            double temp2 = links[i].getTotalweight();
            weight = weight + temp1 + temp2;
        }
        manipulatorWeight = weight;
    }

    /**
     * @return the payload
     */
    public double getPayload() {
        return payload;
    }

    /**
     * @return the reach
     */
    public double getReach() {
        return reach;
    }

    public int getLimitingJoint() {
        return limitingJoint;
    }

}
