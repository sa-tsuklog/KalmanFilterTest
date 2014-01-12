/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package kalmanfiltertest;

import MyLib.MyMath.Matrix;
import MyLib.MyMath.MatrixInvalidOperationException;
import MyLib.Util.MyUtil;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author sa
 */
public class Experiment {
    static final int length = 5000;
    static final double interval = 0.01;
    
    static final double accelSigma = 1;
    static final double speedSigma = 0.1;
    static final double posSigma = 0.1;
    
    static final double accelBias = 0.0;
    
    Matrix estimatedErrorP;
    Matrix inputErrorQ;
    Matrix observationErrorR;
    
    Random rnd;
    
    public static void main(String[] args) {
        Experiment exp = new Experiment();
    }

    public Experiment() {
        double sum=0.0;
        int repeat = 1;
        
        try {
            PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter("1axis.txt")));
            
            for (int i = 0; i < repeat; i++) {
                sum += exp(true,pw);
            }
            
            pw.close();
        } catch (IOException iOException) {
            
        }
        MyLib.Util.MyUtil util = new MyUtil();
        util.exec("gnuplot 1axis.plt");
        System.out.println(Math.sqrt(sum/repeat));
    }
    private double exp(boolean print,PrintWriter pw){
        rnd = new Random(System.nanoTime());

        double[] trueAccel = new double[length];
        double[] trueSpeed = new double[length];
        double[] truePos = new double[length];

        double[] accel = new double[length];
        double[] speed = new double[length];
        double[] pos = new double[length];

        double[] insSpeed = new double[length];
        double[] insPos = new double[length];

        double[] observedSpeed = new double[length];
        double[] observedPos = new double[length];

        estimatedErrorP = new Matrix(2,2,0);
        inputErrorQ = new Matrix(1);
        inputErrorQ.set(0, 0, accelSigma*accelSigma);

        observationErrorR = new Matrix(2);
        observationErrorR.set(0, 0, speedSigma*speedSigma);
        observationErrorR.set(1, 1, posSigma*posSigma);

        initAccel(trueAccel);
        calcSpeed(trueSpeed, trueAccel);
        calcPos(truePos, trueSpeed, trueAccel);

        calcAccel(accel, trueAccel);

        calcSpeed(insSpeed, accel);
        calcPos(insPos, insSpeed, accel);

        calcObservedSpeed(observedSpeed, trueSpeed);
        calcObservedPos(observedPos, truePos);

        pos[0] = 0;
        speed[0] = 0;

        for (int i = 1; i < length; i++) {
            System.out.println(estimatedErrorP);
            System.out.println("");
            
            predict(pos, speed, accel, i);

            if(print){
                //pw.println(i + " " + (pos[i] - truePos[i]) + " " + Math.sqrt(estimatedErrorP.getNum(0, 0))+ " "+ Math.sqrt(estimatedErrorP.getNum(0, 0))*-1);
                pw.println(i+" "+pos[i]+" "+truePos[i]+" "+observedPos[i]+" "+insPos[i]);
                //pw.println(i+" "+speed[i]+" "+trueSpeed[i]+" "+observedSpeed[i]+" "+insSpeed[i]);
            }

            if (i % 10 == 0) {
                update(pos, speed, accel, observedPos, observedSpeed, i);
            }
        }

        double squareSum = 0.0;
        for (int i = 200; i < length; i++) {
            squareSum += Math.pow(pos[i] - truePos[i], 2);
        }
        squareSum /= (length-200);

        
        return squareSum;
    }
    
    private void initAccel(double[] outAccel){
        for (int i = 0; i < 100; i++) {
            outAccel[i] = 1.0;
        }
        for (int i = 100; i < 200; i++) {
            outAccel[i] = 0.0;
        }
        for (int i = 200; i < 300; i++) {
            outAccel[i] = -1.0;
        }
        for (int i = 300; i < 350; i++) {
            outAccel[i] = -4.0;
        }
        for (int i = 350; i < 400; i++) {
            outAccel[i] = 4.0;
        }
        for (int i = 400; i < outAccel.length; i++) {
            outAccel[i] = 0;
        }
    }
    private void calcSpeed(double[] outSpeed,double[] accel){
        outSpeed[0]=accel[0]*interval;
        for (int i = 1; i < length; i++) {
            outSpeed[i] = outSpeed[i-1]+accel[i]*interval;
        }
    }
    private void calcPos(double[] outPos,double[]speed,double[] accel){
        outPos[0]=speed[0]*interval + 1.0/2*accel[0]*interval*interval;
        for (int i = 1; i < length; i++) {
            outPos[i] = outPos[i-1] + speed[i]*interval + 1.0/2*accel[i]*interval*interval;
        }
    }
    
    private void calcAccel(double[] outAccel,double[] trueAccel){
        for (int i = 0; i < length; i++) {
            outAccel[i] = trueAccel[i] + rnd.nextGaussian()*accelSigma + accelBias;
        }
    }
    
    private void calcObservedSpeed(double[] outObservedSpeed,double[] trueSpeed){
        for (int i = 0; i < length; i++) {
            outObservedSpeed[i] = trueSpeed[i] + rnd.nextGaussian()*speedSigma;
        }
    }
    private void calcObservedPos(double[] outObservedPos,double[] truePos){
        for (int i = 0; i < length; i++) {
            outObservedPos[i] = truePos[i] + rnd.nextGaussian()*posSigma;
        }
    }
    
    
    private void predict(double[] outPos,double[] outSpeed,double[] accel,int index){
        Matrix matF = new Matrix(2);
        matF.set(1, 0, interval);
        
        Matrix matG = new Matrix(2, 1, 0);
        matG.set(0, 0, interval*interval/2);
        matG.set(0, 1, interval);
        
        Matrix matX = new Matrix(2, 1, 0);
        matX.set(0, 0, outPos[index-1]);
        matX.set(0, 1, outSpeed[index-1]);
        
        Matrix matU = new Matrix(1, 1, 0);
        matU.set(0, 0, accel[index-1]);
        try {
            Matrix matXnew = matF.mul(matX).add(matG.mul(matU));
            
            estimatedErrorP= matF.mul(estimatedErrorP).mul(matF.transposition()).add(matG.mul(inputErrorQ).mul(matG.transposition()));
            
            outPos[index] = matXnew.getNum(0, 0);
            outSpeed[index]=matXnew.getNum(1, 0);
        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(Experiment.class.getName()).log(Level.SEVERE, null, ex);
        }

    }
    private void update(double[] outPos,double[] outSpeed,double[] accel,double[] observedPos,double[] observedSpeed,int index){
        Matrix matZ = new Matrix(2, 1, 0);
        matZ.set(0, 0, observedPos[index]);
        matZ.set(0, 1, observedSpeed[index]);
        
        Matrix matX = new Matrix(2,1,0);
        matX.set(0, 0, outPos[index]);
        matX.set(0, 1, outSpeed[index]);
        
        Matrix matI = new Matrix(2);
        
        try {
            Matrix matE = matZ.sub(matX);
            
            Matrix matS = observationErrorR.add(estimatedErrorP);
            Matrix matK = estimatedErrorP.mul(matS.inverse());
            
            Matrix matNewX = matX.add(matK.mul(matE));
            estimatedErrorP = (matI.sub(matK)).mul(estimatedErrorP);
            
            outPos[index] = matNewX.getNum(0, 0);
            outSpeed[index]=matNewX.getNum(1, 0);
        } catch (MatrixInvalidOperationException ex) {
            Logger.getLogger(Experiment.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    
}
