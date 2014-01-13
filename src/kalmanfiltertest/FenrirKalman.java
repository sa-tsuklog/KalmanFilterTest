/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package kalmanfiltertest;

import MyLib.MyMath.Matrix;
import MyLib.MyMath.Quaternion;
import MyLib.Util.MyUtil;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Random;

/**
 *
 * @author sa
 */
public class FenrirKalman {

    /**
     * @param args the command line arguments
     */
    double r = 100;
    double v = v=2*r*Math.PI/10;
    Random rnd;
    
    static final double earthRadius=6378137;
    static final double decentering=0.0818191908426;
    
    public static void main(String[] args) {
        FenrirKalman fk= new FenrirKalman();
    }

    public FenrirKalman() {
        sim();
    }
    private void serial(){
        
    }
    
    private void sim(){
        rnd = new Random(System.nanoTime());
        //rnd = new Random(0);

        //Quaternion velocity = new Quaternion(0, v, 0, 0);
        Quaternion velocity = new Quaternion(0, 0.2, v, 0);
        //Quaternion position = GAINS.KalmanFilter.angleToPosition(-100.0 / earthRadius, 0, 0);
        Quaternion position = GAINS.KalmanFilter.angleToPosition(0, 100.0 / earthRadius, 0);
        //Quaternion attitude = new Quaternion(1, 0, 0, 0);
        Quaternion attitude = new Quaternion(Math.cos(Math.PI/2/2), 0, 0, Math.sin(Math.PI/2/2));

        double height = 0;

        Matrix errorP = GAINS.KalmanFilter.initializeP(r, v);

        GAINS.KalmanFilter kf = new GAINS.KalmanFilter(1.0 / 100, velocity, position, attitude, height, errorP);

        Accelometer accel = new Accelometer();
        Gyro gyro = new Gyro();
        SimGps gps = new SimGps();

        SimTrueState trueState = new SimTrueState();

        try {
            PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter("test.txt")));

            double squareSum=0;
            int repeat = 3000;

            
            for (int i = 0; i < repeat; i++) {
                kf.predict(accel.getAccel(), gyro.getRate());
                
                if (i % 10 == 0) {
                    Quaternion gpsPos = gps.getPosition(r, i);
                    Quaternion gpsVel = gps.getVelocity(v, i);
                    
                    //kf.update(gpsVel, gpsPos, 0.001*rnd.nextGaussian());
                    kf.update(gpsVel, gpsPos, 0);
                    
                    //pos
                    pw.println(i + " " + GAINS.KalmanFilter.positionToLambda(kf.getPos())*earthRadius + " " + GAINS.KalmanFilter.positionToPhi(kf.getPos())*earthRadius
                        +" "+GAINS.KalmanFilter.positionToLambda(gpsPos)*earthRadius+" "+GAINS.KalmanFilter.positionToPhi(gpsPos)*earthRadius
                        +" "+GAINS.KalmanFilter.positionToLambda(trueState.getPosition(r, i))*earthRadius+" "+ GAINS.KalmanFilter.positionToPhi(trueState.getPosition(r, i))*earthRadius);
                    
                    //pos difference
                    //pw.println(i+" "+posDifference(kf.getPos(), trueState.getPosition(r, i))+" "+posDifference(gpsPos, trueState.getPosition(r, i)));
                    
                    //speed
//                    pw.println(i+" "+kf.getSpeed().getX()+" "+kf.getSpeed().getY()
//                            +" "+gpsVel.getX()+" "+gpsVel.getY()
//                            +" "+trueState.getVelocity(v, i).getX()+" "+trueState.getVelocity(v, i).getY());
                    
                    
                    squareSum += posDifference(kf.getPos(), trueState.getPosition(r, i));
                }
//                
                //pw.println(i+" "+kf.getAttitude().calcRadHeading()+" "+kf.getAttitude().calcRadPitch()+" "+kf.getAttitude().calcRadRole());

            }
            
            pw.close();
            
            System.out.println("pos error:"+Math.sqrt(squareSum)/repeat*1000);
            
            MyUtil util = new MyUtil();

            util.exec("gnuplot test.plt");

        } catch (IOException iOException) {
            iOException.printStackTrace();
        }
    }
    
    private class SimGps{
        double noiseLevel;
        Quaternion biasDrift;
        double biasLevel;
        double biasTimeBeta;
        
        double spdNoiseLevel;
        
        SimGps(){
            noiseLevel=0.1*r;
            //noiseLevel=0;
            biasLevel=0.05*r;
            //biasLevel=0;
            biasTimeBeta=-0.996;
            biasDrift=getNoise(rnd, biasLevel);
            
            spdNoiseLevel = 0.1;
            //spdNoiseLevel = 0;
        }
        public Quaternion getPosition(double r,int i){
            double xpos,ypos;
//            xpos=-r*(Math.cos(2*Math.PI*i/10 /100))+rnd.nextGaussian()*noiseLevel;
//            ypos= r*(Math.sin(2*Math.PI*i/10 /100))+rnd.nextGaussian()*noiseLevel;
            xpos= -r*(Math.sin(-2*Math.PI*i/10 /100))+rnd.nextGaussian()*noiseLevel;
            ypos= r*(Math.cos(-2*Math.PI*i/10 /100))+rnd.nextGaussian()*noiseLevel;

            biasDrift=getBiasNoise(rnd, biasLevel, biasDrift, biasTimeBeta);
            xpos+=biasDrift.getX();
            ypos+=biasDrift.getY();
            
            return GAINS.KalmanFilter.angleToPosition(xpos/earthRadius, ypos/earthRadius, 0);
        }
        public Quaternion getVelocity(double v,int i){
            double xvel,yvel;
//            xvel= v*(Math.cos(2.0*Math.PI*i/10.0 /100.0)+rnd.nextGaussian()*spdNoiseLevel);
//            yvel= v*(Math.sin(2.0*Math.PI*i/10.0 /100.0)+rnd.nextGaussian()*spdNoiseLevel);
            xvel =-v * (Math.sin(2 * Math.PI * i / 10 /100)+rnd.nextGaussian()*spdNoiseLevel);
            yvel =v * (Math.cos(2 * Math.PI * i / 10/ 100)+rnd.nextGaussian()*spdNoiseLevel);

            return new Quaternion(0, xvel, yvel, 0);
        }
    }
    private class Accelometer {

        Quaternion accelBase;
        double noiseLevel;
        Quaternion biasDrift;
        double biasLevel;
        double biasTimeBeta;

        Accelometer() {
            accelBase = new Quaternion(0, 0, v * v / r, -9.8);
            //accelBase= new Quaternion(0,0,v*v/r,0);
            noiseLevel = 0.03 * v * v / r;
            //noiseLevel = 0.00;
            biasLevel = 0.03 * v * v / r;
            //biasLevel = 0.0;
            biasTimeBeta = -0.994;
            biasDrift = getNoise(rnd, biasLevel);
        }

        public Quaternion getAccel() {
            Quaternion whiteNoise;
            whiteNoise = getNoise(rnd, noiseLevel);
            biasDrift = getBiasNoise(rnd, biasLevel, biasDrift, biasTimeBeta);
            return accelBase.add(whiteNoise).add(biasDrift);
        }

        public Quaternion getBias() {
            return biasDrift;
        }
    }
    private class Gyro {

        Quaternion gyroBase;
        double noiseLevel;
        double biasLevel;
        double biasTimeBeta;
        Quaternion biasDrift;

        Gyro() {
            gyroBase = new Quaternion(0, 0, 0, 2 * Math.PI / 10);
            noiseLevel = 0.03 * 2 * Math.PI;
            //noiseLevel = 0.0;
            biasLevel = 0.03 * 2 * Math.PI;
            //biasLevel = 0;
            biasTimeBeta = -0.994;
            biasDrift = getNoise(rnd, biasLevel);
        }

        public Quaternion getRate() {
            Quaternion whiteNoise;
            whiteNoise = getNoise(rnd, noiseLevel);
            biasDrift = getBiasNoise(rnd, biasLevel, biasDrift, biasTimeBeta);
            return gyroBase.add(whiteNoise).add(biasDrift);
        }

        public Quaternion getBias() {
            return biasDrift;
        }
    }
    private Quaternion getBiasNoise(Random rnd, double stdDev, Quaternion previous,double beta){
        double x=(rnd.nextDouble()-0.5)*stdDev;
        double y=(rnd.nextDouble()-0.5)*stdDev;
        double z=(rnd.nextDouble()-0.5)*stdDev;

        x-=beta*previous.getX();
        y-=beta*previous.getY();
        z-=beta*previous.getZ();

        return new Quaternion(0, x, y, z);
    }
    private Quaternion getNoise(Random rnd,double stdDev){
        double x=(rnd.nextDouble()-0.5)*stdDev;
        double y=(rnd.nextDouble()-0.5)*stdDev;
        double z=(rnd.nextDouble()-0.5)*stdDev;

        return new Quaternion(0, x, y, z);
    }
    private class SimTrueState{
        Quaternion accelBase;
        Quaternion gyroBase;
        public SimTrueState() {
            accelBase = new Quaternion(0, 0, v * v / r, -9.8);
            gyroBase = new Quaternion(0, 0, 0, 2 * Math.PI / 10);
        }
        
        public Quaternion getAccel(){
            return accelBase;
        }
        public Quaternion getRate(){
            return gyroBase;
        }
        public double getPitch(){
            return 0;
        }
        public double getRole(){
            return 0;
        }
        public double getHeading(int i){
            return Math.sin(2*Math.PI*i/10 / 100);
        }
        public Quaternion getPosition(double r,int i){
            double xpos, ypos;
//            xpos = -r * (Math.cos(2 * Math.PI * i / 10 / 100));
//            ypos = r * (Math.sin(2 * Math.PI * i / 10 / 100));
            xpos= -r*(Math.sin(-2*Math.PI*i/10 /100));
            ypos= r*(Math.cos(-2*Math.PI*i/10 /100));

            return GAINS.KalmanFilter.angleToPosition(xpos / earthRadius, ypos / earthRadius, 0);
        }
        public Quaternion getVelocity(double v,int i){
            double xvel, yvel;
//            xvel = v * (Math.cos(2 * Math.PI * i / 10 /100));
//            yvel = v * (Math.sin(2 * Math.PI * i / 10/ 100));
            xvel =-v * (Math.sin(2 * Math.PI * i / 10 /100));
            yvel =v * (Math.cos(2 * Math.PI * i / 10/ 100));
            
            return new Quaternion(0, xvel, yvel, 0);
        }
        
    }
    double posDifference(Quaternion pos,Quaternion truePos){
        double x = GAINS.KalmanFilter.positionToLambda(pos);
        double y = GAINS.KalmanFilter.positionToPhi(pos);
        
        double trueX = GAINS.KalmanFilter.positionToLambda(truePos);
        double trueY = GAINS.KalmanFilter.positionToPhi(truePos);
        
        return Math.pow(x-trueX,2)+Math.pow(y-trueY, 2);
    }
}
