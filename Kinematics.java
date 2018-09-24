/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package kinematics;

import Jama.Matrix;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import java.text.DecimalFormat;
import java.util.Arrays;

/**
 *
 * @author Emil
 */
public class Kinematics {

    private double pi = Math.PI;

    private double[][] DH = new double[][]{{0, pi / 2.0, -0.180, 0},
    {1.340, 0, 0, -pi / 2.0},
    {1.340, 0, 0, 0},
    {0.000, -pi / 2.0, 0, pi / 2.0},
    {0.500, 0, -0.290, 0},
    {0.500, 0, -0.067, 0}};

   

    private long startTime = 0;

    private int maxIt = 1000;

    public Kinematics(double[] adjustableLinksLengths) {
        //DH = new double[][] {{0.025, -pi/2.0, 0.4, 0},{0.455,0,0,0},{0.35,-pi/2.0,0,-pi/2.0},{0,pi/2.0,0.420,0},{0,-pi/2.0,0,0},{0,0,0.08,0}};
        setLinkLength(adjustableLinksLengths[0], 1);
        setLinkLength(adjustableLinksLengths[1], 2);
        setLinkLength(adjustableLinksLengths[2], 4);
        setLinkLength(adjustableLinksLengths[3], 5);
    }

    public void setDHwithPrint(double[][] inputDH) {
        setDH(inputDH);
        String printString = Arrays.deepToString(getDH());
        System.out.println("New DH: " + printString);
    }

    public void setLinkLength(double value, int linkNr) {
        DH[linkNr][0] = value;
    }

    public void setParamDH(double val, int row, int col) {
        DH[row][col] = val;
    }

    public double[][][] calculateForwardKin(double[] jointValues) {
        double[][] transformationMatrix;

        double[] input0 = {getDH()[0][0], getDH()[0][1], getDH()[0][2], jointValues[0] + getDH()[0][3]};
        double[][] T01 = DH2T(input0);

        double[] input1 = {getDH()[1][0], getDH()[1][1], getDH()[1][2], jointValues[1] + getDH()[1][3]};
        double[][] T12 = DH2T(input1);

        double[] input2 = {getDH()[2][0], getDH()[2][1], getDH()[2][2], jointValues[2] + getDH()[2][3]};
        double[][] T23 = DH2T(input2);

        double[] input3 = {getDH()[3][0], getDH()[3][1], getDH()[3][2], jointValues[3] + getDH()[3][3]};
        double[][] T34 = DH2T(input3);

        double[] input4 = {getDH()[4][0], getDH()[4][1], getDH()[4][2], jointValues[4] + getDH()[4][3]};
        double[][] T45 = DH2T(input4);

        double[] input5 = {getDH()[5][0], getDH()[5][1], getDH()[5][2], jointValues[5] + getDH()[5][3]};
        double[][] T56 = DH2T(input5);


        double[][] T02 = matrixMult(T01, T12);

        double[][] T03 = matrixMult(T02, T23);

        double[][] T04 = matrixMult(T03, T34);

        double[][] T05 = matrixMult(T04, T45);

        double[][] T06 = matrixMult(T05, T56);

        double[] Z0 = {0, 0, 1};
        //printMatrix(Z01);
        double[] Z1 = getZvector(T01);

        double[] Z2 = getZvector(T02);

        double[] Z3 = getZvector(T03);

        double[] Z4 = getZvector(T04);

        double[] Z5 = getZvector(T05);

        double[][] Zs = {Z0, Z1, Z2, Z3, Z4, Z5};

        double[] P0 = {0, 0, 0};
        double[] P1 = getPvector(T01);
        double[] P2 = getPvector(T02);
        double[] P3 = getPvector(T03);
        double[] P4 = getPvector(T04);
        double[] P5 = getPvector(T05);
        double[] P6 = getPvector(T06);
        //printVector(P6);
        double[][] Ps = {P0, P1, P2, P3, P4, P5, P6};
        //printMatrix(P6);

        double[][] jacobi = generateJacobi(Ps, Zs);
        //printMatrix(jacobi);

        transformationMatrix = T06;
        double[][][] returnMatrix = {jacobi, transformationMatrix};
        //printArrayofMatrix(returnMatrix);
        //printMatrix(T06);

        return returnMatrix;
    }

    public double[][] getJointCentersWorldFrame(double[] jointValues) {
        double[] input0 = {getDH()[0][0], getDH()[0][1], getDH()[0][2], jointValues[0] + getDH()[0][3]};
        double[][] T01 = DH2T(input0);

        double[] input1 = {getDH()[1][0], getDH()[1][1], getDH()[1][2], jointValues[1] + getDH()[1][3]};
        double[][] T12 = DH2T(input1);

        double[] input2 = {getDH()[2][0], getDH()[2][1], getDH()[2][2], jointValues[2] + getDH()[2][3]};
        double[][] T23 = DH2T(input2);

        double[] input3 = {getDH()[3][0], getDH()[3][1], getDH()[3][2], jointValues[3] + getDH()[3][3]};
        double[][] T34 = DH2T(input3);

        double[] input4 = {getDH()[4][0], getDH()[4][1], getDH()[4][2], jointValues[4] + getDH()[4][3]};
        double[][] T45 = DH2T(input4);

        double[] input5 = {getDH()[5][0], getDH()[5][1], getDH()[5][2], jointValues[5] + getDH()[5][3]};
        double[][] T56 = DH2T(input5);

        double[][] T02 = matrixMult(T01, T12);
        double[][] T03 = matrixMult(T02, T23);
        double[][] T04 = matrixMult(T03, T34);
        double[][] T05 = matrixMult(T04, T45);
        double[][] T06 = matrixMult(T05, T56);

        double[][] T46 = matrixMult(T45, T56);
        double[][] T36 = matrixMult(T34, T46);
        double[][] T26 = matrixMult(T23, T36);
        double[][] T16 = matrixMult(T12, T26);

        double[] P0 = {0, 0, 0};
        double[] P1 = getPvector(T01);
        double[] P2 = getPvector(T02);
        double[] P3 = getPvector(T03);
        double[] P4 = getPvector(T04);
        double[] P5 = getPvector(T05);
        double[] P6 = getPvector(T06);
        //printVector(P6);
        double[][] Ps = {P0, P1, P2, P3, P4, P5, P6};
        return Ps;
    }

    public double[][] getPx6WorldFrame(double[] jointValues) {
        double[] input0 = {getDH()[0][0], getDH()[0][1], getDH()[0][2], jointValues[0] + getDH()[0][3]};
        double[][] T01 = DH2T(input0);

        double[] input1 = {getDH()[1][0], getDH()[1][1], getDH()[1][2], jointValues[1] + getDH()[1][3]};
        double[][] T12 = DH2T(input1);

        double[] input2 = {getDH()[2][0], getDH()[2][1], getDH()[2][2], jointValues[2] + getDH()[2][3]};
        double[][] T23 = DH2T(input2);

        double[] input3 = {getDH()[3][0], getDH()[3][1], getDH()[3][2], jointValues[3] + getDH()[3][3]};
        double[][] T34 = DH2T(input3);

        double[] input4 = {getDH()[4][0], getDH()[4][1], getDH()[4][2], jointValues[4] + getDH()[4][3]};
        double[][] T45 = DH2T(input4);

        double[] input5 = {getDH()[5][0], getDH()[5][1], getDH()[5][2], jointValues[5] + getDH()[5][3]};
        double[][] T56 = DH2T(input5);

        double[][] T02 = matrixMult(T01, T12);
        double[][] T03 = matrixMult(T02, T23);
        double[][] T04 = matrixMult(T03, T34);
        double[][] T05 = matrixMult(T04, T45);
        double[][] T06 = matrixMult(T05, T56);

        double[][] T46 = matrixMult(T45, T56);
        double[][] T36 = matrixMult(T34, T46);
        double[][] T26 = matrixMult(T23, T36);
        double[][] T16 = matrixMult(T12, T26);

        double[] P06 = getPvector(T06);
        double[] P16 = vectorSubtraction(P06, getPvector(T01));
        double[] P26 = vectorSubtraction(P06, getPvector(T02));
        double[] P36 = vectorSubtraction(P06, getPvector(T03));
        double[] P46 = vectorSubtraction(P06, getPvector(T04));
        double[] P56 = vectorSubtraction(P06, getPvector(T05));

        double[][] Px6 = {P06, P16, P26, P36, P46, P56};
        return Px6;
    }

    public double[][] getLinkZVectorsFromJointValues(double[] jointValues) {
        double[] input0 = {getDH()[0][0], getDH()[0][1], getDH()[0][2], jointValues[0] + getDH()[0][3]};
        double[][] T01 = DH2T(input0);

        double[] input1 = {getDH()[1][0], getDH()[1][1], getDH()[1][2], jointValues[1] + getDH()[1][3]};
        double[][] T12 = DH2T(input1);

        double[] input2 = {getDH()[2][0], getDH()[2][1], getDH()[2][2], jointValues[2] + getDH()[2][3]};
        double[][] T23 = DH2T(input2);

        double[] input3 = {getDH()[3][0], getDH()[3][1], getDH()[3][2], jointValues[3] + getDH()[3][3]};
        double[][] T34 = DH2T(input3);

        double[] input4 = {getDH()[4][0], getDH()[4][1], getDH()[4][2], jointValues[4] + getDH()[4][3]};
        double[][] T45 = DH2T(input4);

        double[] input5 = {getDH()[5][0], getDH()[5][1], getDH()[5][2], jointValues[5] + getDH()[5][3]};
        double[][] T56 = DH2T(input5);

        double[][] T02 = matrixMult(T01, T12);

        double[][] T03 = matrixMult(T02, T23);

        double[][] T04 = matrixMult(T03, T34);

        double[][] T05 = matrixMult(T04, T45);

        double[][] T06 = matrixMult(T05, T56);

        double[] p0 = {0, 0, 0};
        double[] p1 = getPvector(T01);
        double[] p2 = getPvector(T02);
        double[] p3 = getPvector(T03);
        double[] p4 = getPvector(T04);
        double[] p5 = getPvector(T05);
        double[] p6 = getPvector(T06);
        
        double[] Z1 = vectorSubtraction(p1, p0);
        double[] Z2 = vectorSubtraction(p2, p1);
        double[] Z3 = vectorSubtraction(p3, p2);
        double[] Z4 = vectorSubtraction(p4, p3);
        double[] Z5 = vectorSubtraction(p5, p4);
        double[] Z6 = vectorSubtraction(p6, p5);
        
        double[][] Zs = {Z1, Z2, Z3, Z4, Z5, Z6};
        return Zs;
    }

    public double[][] getJointZVectorsFromJointValues(double[] jointValues) {
        double[] input0 = {getDH()[0][0], getDH()[0][1], getDH()[0][2], jointValues[0] + getDH()[0][3]};
        double[][] T01 = DH2T(input0);

        double[] input1 = {getDH()[1][0], getDH()[1][1], getDH()[1][2], jointValues[1] + getDH()[1][3]};
        double[][] T12 = DH2T(input1);

        double[] input2 = {getDH()[2][0], getDH()[2][1], getDH()[2][2], jointValues[2] + getDH()[2][3]};
        double[][] T23 = DH2T(input2);

        double[] input3 = {getDH()[3][0], getDH()[3][1], getDH()[3][2], jointValues[3] + getDH()[3][3]};
        double[][] T34 = DH2T(input3);

        double[] input4 = {getDH()[4][0], getDH()[4][1], getDH()[4][2], jointValues[4] + getDH()[4][3]};
        double[][] T45 = DH2T(input4);

        double[] input5 = {getDH()[5][0], getDH()[5][1], getDH()[5][2], jointValues[5] + getDH()[5][3]};
        double[][] T56 = DH2T(input5);

        double[][] T02 = matrixMult(T01, T12);

        double[][] T03 = matrixMult(T02, T23);

        double[][] T04 = matrixMult(T03, T34);

        double[][] T05 = matrixMult(T04, T45);

        double[][] T06 = matrixMult(T05, T56);

        double[] Z0 = {0, 0, 1};
        //printMatrix(Z01);
        double[] Z1 = getZvector(T01);

        double[] Z2 = getZvector(T02);

        double[] Z3 = getZvector(T03);

        double[] Z4 = getZvector(T04);

        double[] Z5 = getZvector(T05);

        double[][] Zs = {Z0, Z1, Z2, Z3, Z4, Z5};
        return Zs;
    }



    public double[] calculateInvKin(double[] qi, double[][] Td, int[] konfigBits) {
        double[] qk = qi;

        
        startTime = System.nanoTime();
        System.out.println("Setting start time");
        

        double[][] Rd = getPartOfMatrix(Td, 0, 3, 0, 3);
        
        double[] ep = {1, 1, 1};
        double[] eo = {1, 1, 1};

        //avoid singularities
        int ex = 0;

        while ((((Math.abs(ep[0]) + Math.abs(ep[1]) + Math.abs(ep[2])) > 0.0005)
                || (Math.abs(eo[0]) + Math.abs(eo[1]) + Math.abs(eo[2])) > 0.000005) && ex < 1000) {

            
            double[][][] temp = calculateForwardKin(qk);
            double[][] Jk = temp[0];
            double[][] Tk = temp[1];

            double[][] Rk = getPartOfMatrix(Tk, 0, 3, 0, 3);

            double[][] Rktrans = transposeMatrix(Rk);

            double[][] R = matrixMult(Rd, Rktrans);

            double[] posTd = getPvector(Td);
            double[] posTk = getPvector(Tk);

            ep = matrixMinus(posTd, posTk);

            //printVector(ep);
            eo = new double[]{(0.5 * (R[2][1] - R[1][2])), (0.5 * (R[0][2] - R[2][0])), (0.5 * (R[1][0] - R[0][1]))};

            //printVector(eo);
            double Kp = 0.6229;
            double Ko = 0.8046;

            
            double[] temp2 = multiplyVectorByNumber(ep, Kp);
            double[] temp3 = multiplyVectorByNumber(eo, Ko);
            double[][] temp4 = {{temp2[0]}, {temp2[1]}, {temp2[2]}, {temp3[0]}, {temp3[1]}, {temp3[2]}};
            
            Matrix A = new Matrix(Jk);
            Matrix inverse = PseudoInverse.pinv(A);
            double[][] Jkinv = getMatrixFromJama(inverse);

            
            double[][] dp = matrixMult(Jkinv, temp4);

            //printMatrix(dp);
            for (int a = 0; a < 6; a++) {
                qk[a] = qk[a] + dp[a][0];
                if (qk[a] > pi) {
                    qk[a] = qk[a] - pi;
                } else if (qk[a] < -pi) {
                    qk[a] = qk[a] + pi;
                }

            }
            //printVector(qk);

            if (konfigBits != null) {
                //System.out.println("KonfigBits");
                if (konfigBits[0] == 1) {
                    if (qk[2] < 0) {
                        qk[2] = -qk[2];
                        qk[1] = -qk[1];
                        qk[3] = -qk[3];
                    }

                } else if (konfigBits[0] == 0) {
                    if (qk[2] > 0) {
                        qk[2] = -qk[2];
                        qk[1] = -qk[1];
                        qk[3] = -qk[3];
                    }
                }
                if (konfigBits[1] == 1) {
                    if (qk[5] < 0) {
                        qk[5] = -qk[5];
                        qk[4] = -qk[4];
                    }
                } else if (konfigBits[1] == 0) {
                    if (qk[5] > 0) {
                        qk[5] = -qk[5];
                        qk[4] = -qk[4];
                    }
                }
            }

            //printVector(qk);
            ex++;
        }

        double[][][] temp7 = calculateForwardKin(qk);
        
        System.out.println("Number of iterations: " + ex);
        System.out.println("Elapsed time in us: " + ((System.nanoTime() - startTime) / 1000) + "\n");
        if (ex == 1000) {
            System.out.println("Probably at singularity");
        }
        //printVector(eo);
        return qk;
    }

    public int calculateInvKinGain(double[] qi, double[][] Td, int[] konfigBits, double[] gain) {
        double[] qk = qi;

        
        startTime = System.nanoTime();
        

        double[][] Rd = getPartOfMatrix(Td, 0, 3, 0, 3);
       
        double[] ep = {1, 1, 1};
        double[] eo = {1, 1, 1};

        //avoid singularities
        int ex = 0;

        double Kp = gain[0];
        double Ko = gain[1];

        double[][][] temp = calculateForwardKin(qk);
        double[][] Jk = temp[0];
        double[][] Tk = temp[1];

        double[][] Rk = getPartOfMatrix(Tk, 0, 3, 0, 3);

        double[][] Rktrans = transposeMatrix(Rk);

        double[][] R = matrixMult(Rd, Rktrans);

        double[] posTd = getPvector(Td);
        double[] posTk = getPvector(Tk);

        double[] temp2 = multiplyVectorByNumber(ep, Kp);
        double[] temp3 = multiplyVectorByNumber(eo, Ko);
        double[][] temp4 = {{temp2[0]}, {temp2[1]}, {temp2[2]}, {temp3[0]}, {temp3[1]}, {temp3[2]}};

        double[][] dp;
        double[][] Jkinv;
        Matrix A;
        Matrix inverse;
        A = new Matrix(Jk);
        inverse = PseudoInverse.pinv(A);
        Jkinv = getMatrixFromJama(inverse);
        double tempqq;

        while ((((Math.abs(ep[0]) + Math.abs(ep[1]) + Math.abs(ep[2])) > 0.0005)
                || (Math.abs(eo[0]) + Math.abs(eo[1]) + Math.abs(eo[2])) > 0.000005) && ex < getMaxIt()) {

            //qk[1]=qk[1]+qk[3];
            temp = calculateForwardKin(qk);
            Jk = temp[0];
            Tk = temp[1];

            Rk = getPartOfMatrix(Tk, 0, 3, 0, 3);

            Rktrans = transposeMatrix(Rk);

            R = matrixMult(Rd, Rktrans);

            posTd = getPvector(Td);
            posTk = getPvector(Tk);

            ep = matrixMinus(posTd, posTk);

            //printVector(ep);
            eo = new double[]{(0.5 * (R[2][1] - R[1][2])), (0.5 * (R[0][2] - R[2][0])), (0.5 * (R[1][0] - R[0][1]))};

            
            temp2 = multiplyVectorByNumber(ep, Kp);
            temp3 = multiplyVectorByNumber(eo, Ko);
            temp4 = new double[][]{{temp2[0]}, {temp2[1]}, {temp2[2]}, {temp3[0]}, {temp3[1]}, {temp3[2]}};
            
            A = new Matrix(Jk);
            inverse = PseudoInverse.pinv(A);
            Jkinv = getMatrixFromJama(inverse);

            
            dp = matrixMult(Jkinv, temp4);

            //printMatrix(dp);
            for (int a = 0; a < 6; a++) {
                qk[a] = qk[a] + dp[a][0];
                while (qk[a] > pi || qk[a] < -pi) {
                    //System.out.println("qk[a]: "+qk[a]);
                    if (qk[a] > pi) {
                        tempqq = qk[a];
                        qk[a] = tempqq - pi;
                    } else if (qk[a] < -pi) {
                        tempqq = qk[a];
                        qk[a] = tempqq + pi;
                    }
                    if (qk[a] > 200 || qk[a] < -200) {

                        System.out.println("Q[" + a + "] over 100: " + qk[a]);
                        System.out.println("It: " + ex + "  dp: ");
                        
                        qk[a] = 0.1;
                    }
                }

            }
            //printVector(qk);

            if (konfigBits != null) {
                //System.out.println("KonfigBits");
                if (konfigBits[0] == 1) {
                    if (qk[2] < 0) {
                        qk[2] = -qk[2];
                        qk[1] = -qk[1];
                        qk[3] = -qk[3];
                    }

                } else if (konfigBits[0] == 0) {
                    if (qk[2] > 0) {
                        qk[2] = -qk[2];
                        qk[1] = -qk[1];
                        qk[3] = -qk[3];
                    }
                }
                if (konfigBits[1] == 1) {
                    if (qk[5] < 0) {
                        qk[5] = -qk[5];
                        qk[4] = -qk[4];
                    }
                } else if (konfigBits[1] == 0) {
                    if (qk[5] > 0) {
                        qk[5] = -qk[5];
                        qk[4] = -qk[4];
                    }
                }
            }
            if (ex == (getMaxIt() - 1)) {
                System.out.println("Probably at singularity");
                printMatrix(temp[1]);
                printVector(qk);
                printMatrix(temp4);
                printMatrix(Td);
            }
            //printVector(qk);
            ex++;
        }

        double[][][] temp7 = calculateForwardKin(qk);
        
        System.out.println("Number of iterations: " + ex);
        if (ex == getMaxIt()) {
            System.out.println("Probably at singularity");
            printMatrix(temp7[1]);
            printVector(qk);
            printMatrix(temp4);
            printMatrix(Td);

            ex = 1000000;
        }
        //System.out.println("\n");
        //printVector(eo);
        return ex;
    }

    public int calcCost(double[] input) {
        int cost = 0;
        cost = (int) (100 * Math.abs(0.5 - input[0]));
        cost = cost + (int) (100 * Math.abs(0.5 - input[1]));
        return cost;
    }

    public double[] calculateInvKin2(double[] qi, double[][] Td, double[] konfigBits) {
        double[] qk = qi;
        double l0 = getDH()[0][2];
        double l1 = getDH()[1][0];
        double l2 = getDH()[2][0];
        double l3 = getDH()[3][0];
        double l4 = getDH()[4][0];
        double l5 = getDH()[5][0];

        double[] posTd = getPvector(Td);
        double x = posTd[0];
        System.out.println("x: " + x);
        double y = posTd[1];
        System.out.println("y: " + y);
        double z = posTd[2];
        double horzL = Math.sqrt((x * x) + (y * y));
        System.out.println("Horzl: " + horzL);
        System.out.println("l4: " + l4);

        double q1 = Math.atan2(y, x);
        double tempz = -z - l3 + l0;
        double tempq2 = (tempz) / (2 * l1);
        double q2 = Math.acos(tempq2);
        double q3 = -2 * q2;
        double q4 = q2;
        double q5 = Math.acos(horzL / (2 * l4));
        System.out.println("q5: " + q5);
        double q6 = -2 * q5;

        double[] temppee = {q1, q2, q3, q4, q5, q6};
        qk = temppee;

        return qk;
    }

    public double[][] generateJacobi(double[][] Ps, double[][] Zs) {
        double[][] jacobi = new double[6][6];
        for (int i = 0; i < 6; i++) {
            double[] temp = matrixMinus(Ps[6], Ps[i]);
            double[] cross = crossVectors(Zs[i], temp);
            double[] Z = Zs[i];
            for (int j = 0; j < 3; j++) {
                jacobi[j][i] = cross[j];
                jacobi[j + 3][i] = Z[j];
            }
        }
        return jacobi;
    }

    public double[][] getMatrixFromJama(Matrix inMat) {
        double[][] returnMat = new double[inMat.getRowDimension()][inMat.getColumnDimension()];

        for (int i = 0; i < inMat.getRowDimension(); i++) {
            for (int j = 0; j < inMat.getColumnDimension(); j++) {
                returnMat[i][j] = inMat.get(i, j);
            }
        }
        return returnMat;
    }


    public double[] crossVectors(double[] vec1, double[] vec2) {
        double x = vec1[1] * vec2[2] - vec2[1] * vec1[2];
        double y = vec1[2] * vec2[0] - vec2[2] * vec1[0];
        double z = vec1[0] * vec2[1] - vec2[0] * vec1[1];
        double[] returnVec = {x, y, z};
        return returnVec;
    }

    public double crossVectorsLength(double[] vec1, double[] vec2) {
        double[] vec = crossVectors(vec1, vec2);
        return (Math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]));
    }

    public double calcVectorLength(double[] vec) {
        return (Math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]));
    }

    public double dotVectorLength(double[] vec1, double[] vec2) {
        return Math.abs(vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2]);
    }

    public double[][] getPartOfMatrix(double[][] inMatrix, int rowStart, int rowEnd, int colStart, int colEnd) {
        double[][] outMatrix = new double[rowEnd - rowStart][colEnd - colStart];
        for (int i = colStart; i < colEnd; i++) {
            double[] tempCol = getColumnFromMatrix(inMatrix, i);
            double[] partofTempCol = Arrays.copyOfRange(tempCol, rowStart, rowEnd);
            //printVector(partofTempCol);
            for (int j = 0; j < (rowEnd - rowStart); j++) {
                outMatrix[j][i - colStart] = partofTempCol[j];
            }
        }

        return outMatrix;

    }

    public double[] getDiagonalFromMatrix(double[][] inMat) {
        double[] out = null;
        if (inMat.length == inMat[0].length) {
            out = new double[inMat.length];
            for (int i = 0; i < inMat.length; i++) {
                out[i] = inMat[i][i];
            }

        } else {
            out = new double[]{0};
            System.out.println("Not square");
        }
        return out;
    }

    public double[] multiplyVectorByNumber(double[] inVec, double num) {
        double[] returnArray = new double[inVec.length];

        for (int i = 0; i < inVec.length; i++) {
            returnArray[i] = inVec[i] * num;
        }
        return returnArray;
    }

    public double[] normalizeVector(double[] invec) {
        double lengthsqr = 0;
        double[] newvec = new double[3];
        for (int i = 0; i < invec.length; i++) {
            lengthsqr = lengthsqr + (invec[i] * invec[i]);
        }
        if (lengthsqr > 0.0) {
            double length = Math.sqrt(lengthsqr);
            double oneover = 1 / length;
            newvec = multiplyVectorByNumber(invec, oneover);
        } else {
            newvec = new double[] {0,0,0};
        }
        return newvec;
    }

    public double[] matrixMinus(double[] m1, double[] m2) {
        int m2NumberRows = m1.length;
        //System.out.println(m2NumberRows);
        double[] returnMatrix = new double[m2NumberRows];

        for (int i = 0; i < m2NumberRows; i++) {
            returnMatrix[i] = m1[i] - m2[i];
        }
        return returnMatrix;
    }

    public void printMatrix(double[][] inMat) {
        int m2NumberRows = inMat.length;    // m2 number of rows
        StringBuilder sb = new StringBuilder();

        for (int i = 0; i < m2NumberRows; i++) {
            //String temp = Arrays.toString(inMat[i]);
            String temp = arrayToString(inMat[i]);
            sb.append(temp + "\n");
        }
        System.out.println(sb.toString());
    }

    public String arrayToString(double[] inArray) {
        String outString;
        StringBuilder builder = new StringBuilder();

        for (int i = 0; i < inArray.length; i++) {
            double a = inArray[i];
            DecimalFormat df = new DecimalFormat("#.##");
            builder.append(df.format(a) + "\t");
        }
        return builder.toString();

    }

    public void printArrayofMatrix(double[][][] in) {
        int num = in.length;
        for (int i = 0; i < num; i++) {
            System.out.println("Printing");
            printMatrix(in[i]);
        }
    }

    public static String vectorToString(double[] in) {
        String outString;
        StringBuilder builder = new StringBuilder();

        for (int i = 0; i < in.length; i++) {
            double a = in[i];
            DecimalFormat df = new DecimalFormat("#.####");
            builder.append(df.format(a) + " ");
        }
        return builder.toString();
    }

    public void printVector(double[] inVec) {
        String printString = Arrays.toString(inVec);

        System.out.println(printString);
    }

    public double[] getZvector(double[][] transformM) {

        double[] sush = getColumnFromMatrix(transformM, 2);
        double[] temp = Arrays.copyOfRange(sush, 0, 3);
        double[][] sush2 = {temp};
        double[][] transposed = transposeMatrix(sush2);

        return temp;
    }

    public double[] getPvector(double[][] transformM) {

        double[] sush = getColumnFromMatrix(transformM, 3);
        double[] temp = Arrays.copyOfRange(sush, 0, 3);
        double[][] sush2 = {temp};
        double[][] transposed = transposeMatrix(sush2);

        return temp;
    }

    public double[] getColumnFromMatrix(double[][] inMat, int nRow) {
        double[] returnRow;
        int m1ColLength = inMat[0].length; // m1 number of columns, also the length of the rows
        double[][] transposed = transposeMatrix(inMat);
        returnRow = transposed[nRow];

        return returnRow;
    }

    public double[][] DH2T(double[] inputVec) {
        double[][] transform;
        double a = inputVec[0];
        double alpha = inputVec[1];
        double d = inputVec[2];
        double theta = inputVec[3];

        //transform = translZ(d) * hRotZ(theta) * translX(a) * hRotX(alpha);
        double[][] temp1 = matrixMult(translZ(d), hRotZ(theta));
        double[][] temp2 = matrixMult(temp1, translX(a));
        transform = matrixMult(temp2, hRotX(alpha));

        return transform;
    }

    public double[][] translZ(double d) {
        double[][] transform;
        transform = new double[][]{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, d}, {0, 0, 0, 1}};

        return transform;
    }

    public double[][] hRotZ(double t) {
        double[][] transform = {{cos(t), -sin(t), 0, 0}, {sin(t), cos(t), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
        return transform;
    }

    public double[][] translX(double a) {
        double[][] transform = {{1, 0, 0, a}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
        return transform;
    }

    public double[][] hRotX(double al) {
        double[][] transform = {{1, 0, 0, 0}, {0, cos(al), -sin(al), 0}, {0, sin(al), cos(al), 0}, {0, 0, 0, 1}};
        return transform;
    }

    public double[][] hRotY(double angley) {
        double[][] transform = {{cos(angley), 0, sin(angley), 0}, {0, 1, 0, 0}, {-sin(angley), 0, cos(angley), 0}, {0, 0, 0, 1}};
        return transform;
    }

    public double[][] translY(double y) {
        double[][] transform = {{1, 0, 0, 0}, {0, 1, 0, y}, {0, 0, 1, 0}, {0, 0, 0, 1}};
        return transform;
    }

    public double[][] createTransformFromPosAndEuler(double[] pos, double[] euler) {
        double[][] transform;

        double[][] temp = matrixMult(translX(pos[0]), translY(pos[1]));
        double[][] temp2 = matrixMult(temp, translZ(pos[2]));
        double[][] temp3 = matrixMult(temp2, hRotZ(euler[2]));
        double[][] temp4 = matrixMult(temp3, hRotY(euler[1]));
        double[][] temp5 = matrixMult(temp4, hRotX(euler[0]));

        transform = temp5;
        return transform;
    }

    public double[] getEulerAnglesFromTransformation(double[][] inputTrans) {
        double[][] rotMat = getPartOfMatrix(inputTrans, 0, 3, 0, 3);
        double thetax = Math.atan2(rotMat[2][1], rotMat[2][2]);
        double temp = Math.sqrt((rotMat[2][1] * rotMat[2][1]) + (rotMat[2][2] * rotMat[2][2]));
        double thetay = Math.atan2(-rotMat[2][0], temp);
        double thetaz = Math.atan2(rotMat[1][0], rotMat[0][0]);

        double[] returnVal = {thetax, thetay, thetaz};
        return returnVal;
    }

    public double[][] matrixMult(double[][] m1, double[][] m2) {
        int m1NumberCol = m1[0].length; // m1 number of columns
        int m2NumberRows = m2.length;    // m2 number of rows
        if (m1NumberCol != m2NumberRows) {
            return null; // matrix multiplication is not possible
        }
        int mRRowLength = m1.length;    // m result rows length
        int mRColLength = m2[0].length; // m result columns length
        double[][] mResult = new double[mRRowLength][mRColLength];
        for (int i = 0; i < mRRowLength; i++) {         // rows from m1
            for (int j = 0; j < mRColLength; j++) {     // columns from m2
                for (int k = 0; k < m1NumberCol; k++) { // columns from m1
                    mResult[i][j] += m1[i][k] * m2[k][j];
                }
            }
        }
        return mResult;
    }

    public double[][] transposeMatrix(double[][] m) {
        double[][] temp = new double[m[0].length][m.length];
        for (int i = 0; i < m.length; i++) {
            for (int j = 0; j < m[0].length; j++) {
                temp[j][i] = m[i][j];
            }
        }
        return temp;
    }

    public double[][] generateIdentityMatrix(int dimension) {
        double[][] matrix = new double[dimension][dimension];
        for (int i = 0; i < dimension; i++) {
            matrix[i][i] = 1;
        }
        return matrix;
    }

    /**
     * @return the DH
     */
    public double[][] getDH() {
        return DH;
    }

    /**
     * @param DH the wannabeDH to set
     */
    public void setDH(double[][] wannabeDH) {
        this.DH = wannabeDH;
    }

    public void setSingleDHvalue(double value, int rowpos, int colpos) {
        DH[rowpos][colpos] = value;

        //System.out.println("New DH table:");
        //printMatrix(wannabeDH);
    }

    /**
     * @return the maxIt
     */
    public int getMaxIt() {
        return maxIt;
    }

    /**
     * @param maxIt the maxIt to set
     */
    public void setMaxIt(int maxIt) {
        this.maxIt = maxIt;
    }

    public double[] vectorMidpoint(double[] vec1, double[] vec2) {
        double[] returnvec;
        if (vec1.length == vec2.length) {
            returnvec = new double[vec1.length];
            for (int i = 0; i < vec1.length; i++) {
                double half = (vec2[i] - vec1[i]) / 2;
                returnvec[i] = vec1[i] + half;
            }
        } else {
            System.out.println("Different lengths of vectors.");
            returnvec = null;
        }
        return returnvec;
    }

    public double[] vectorSubtraction(double[] inVec, double[] minusVec) {
        double[] returnvec;
        if (inVec.length == minusVec.length) {
            returnvec = new double[inVec.length];
            for (int i = 0; i < inVec.length; i++) {
                returnvec[i] = inVec[i] - minusVec[i];
            }
        } else {
            System.out.println("Different lengths of vectors.");
            returnvec = null;
        }
        return returnvec;
    }

}
