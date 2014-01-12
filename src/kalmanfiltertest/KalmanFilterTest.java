/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package kalmanfiltertest;

import MyLib.MyMath.Quaternion;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

/**
 *
 * @author sa
 */
public class KalmanFilterTest {

    static final int SAMPLES_LENGTH = 5000;
    static final double SEC_INTERVAL = 0.01;
    static final Quaternion UT_E_CMPS_BASE = new Quaternion(0,1,0,0);
    static final Quaternion MPSPS_E_GRAVITY = new Quaternion(0,0,0,-9.8);
    
    static final Quaternion INITIAL_ATTITUDE  =new Quaternion(1,0,0,0);
    static final Quaternion MPS_INITIAL_SPEED =new Quaternion(0,10,0,0);
    static final Quaternion M_INITIAL_POSITION=new Quaternion(0,0,0,0);
    
    
    public static void main(String[] args) {
        KalmanFilterTest kft = new KalmanFilterTest();
    }

    public KalmanFilterTest() {
        //dummy data generation
        GenerateDummyData dummy = new GenerateDummyData(SAMPLES_LENGTH, SEC_INTERVAL, UT_E_CMPS_BASE, MPSPS_E_GRAVITY, INITIAL_ATTITUDE, MPS_INITIAL_SPEED, M_INITIAL_POSITION);
        
        Quaternion[] mpspsBTrueAccel = new Quaternion[SAMPLES_LENGTH];
        Quaternion[] rpsBTrueGyro = new Quaternion[SAMPLES_LENGTH];
        
        Quaternion[] trueAttitude = new Quaternion[SAMPLES_LENGTH];
        Quaternion[] mpspsETrueAccel=new Quaternion[SAMPLES_LENGTH];
        Quaternion[] mpsETrueSpeed = new Quaternion[SAMPLES_LENGTH];//used as vector.
        Quaternion[] mETruePos = new Quaternion[SAMPLES_LENGTH];//used as vector.
        Quaternion[] utBTrueCmps = new Quaternion[SAMPLES_LENGTH];//used as vector.
        
        dummy.generate(mpspsBTrueAccel, rpsBTrueGyro, trueAttitude, mpspsETrueAccel, mpsETrueSpeed, mETruePos, utBTrueCmps);
        
        
        
        print("dummyImu.txt",trueAttitude,mpspsBTrueAccel,rpsBTrueGyro,utBTrueCmps,mpsETrueSpeed,mETruePos);
        
    }
    
    private void print(String filename, Quaternion[] attitude, Quaternion[] mpspsBAccel, Quaternion[] rpsBGyro, Quaternion[] utBCmps, Quaternion[] mpsESpeed, Quaternion[] mEPos){
        try {
            PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter(filename)));
            
            for (int i = 0; i < SAMPLES_LENGTH; i++) {
                pw.println(i+" "
                        +attitude[i].calcRadHeading()+" "+attitude[i].calcRadPitch()+" "+attitude[i].calcRadRole()+" "
                        +mpspsBAccel[i].getX()+" "+mpspsBAccel[i].getY()+" "+mpspsBAccel[i].getZ()+" "
                        +rpsBGyro[i].getX()+" "+rpsBGyro[i].getY()+" "+rpsBGyro[i].getZ()+" "
                        +utBCmps[i].getX()+" "+utBCmps[i].getY()+" "+utBCmps[i].getZ()+" "
                        +mpsESpeed[i].getX()+" "+mpsESpeed[i].getY()+" "+mpsESpeed[i].getZ()+" "
                        +mEPos[i].getX()+" "+mEPos[i].getY()+" "+mEPos[i].getZ());
                
            }
            pw.close();
        } catch (IOException iOException) {
            
        }
    }
    
    private void  calcAttitude(Quaternion[] outAttitude,Quaternion[] rpsBGyro,int index){
        if(index == 0){
            outAttitude[0] = INITIAL_ATTITUDE;
        }else{
            double dx = Math.sin(rpsBGyro[index-1].getX()*SEC_INTERVAL/2);
            double dy = Math.sin(rpsBGyro[index-1].getY()*SEC_INTERVAL/2);
            double dz = Math.sin(rpsBGyro[index-1].getZ()*SEC_INTERVAL/2);
            double dw = Math.sqrt(1 - (dx*dx + dy*dy + dz*dz));
            
            Quaternion roter = new Quaternion(dw, dx, dy, dz);
            
            
            outAttitude[index] = outAttitude[index-1].mul(roter);
            outAttitude[index] = outAttitude[index].normalize();
        }
    }
    private void calcEAccel(Quaternion[] outMpspsEAccel, Quaternion[] attitude, Quaternion[] mpspsBAccel,int index){
        outMpspsEAccel[index] = attitude[index].mul(mpspsBAccel[index]).mul(attitude[index].con());
    }
    
    private void calcSpeed(Quaternion[] outMpsSpeed, Quaternion[] mpspsEAccel,int index){
        if(index==0){
            outMpsSpeed[0] = MPS_INITIAL_SPEED;
        }else{
            double xSpeed = outMpsSpeed[index-1].getX() + ((mpspsEAccel[index-1].getX()+mpspsEAccel[index].getX())/2-MPSPS_E_GRAVITY.getX())*SEC_INTERVAL;
            double ySpeed = outMpsSpeed[index-1].getY() + ((mpspsEAccel[index-1].getY()+mpspsEAccel[index].getY())/2-MPSPS_E_GRAVITY.getY())*SEC_INTERVAL;
            double zSpeed = outMpsSpeed[index-1].getZ() + ((mpspsEAccel[index-1].getZ()+mpspsEAccel[index].getZ())/2-MPSPS_E_GRAVITY.getZ())*SEC_INTERVAL;
            
            outMpsSpeed[index] = new Quaternion(0, xSpeed, ySpeed, zSpeed);
        }
    }
    
    private void calcPos(Quaternion[] outMPos, Quaternion[] mpsSpeed, Quaternion[] mpspsEAccel,int index){
        if(index==0){
            outMPos[0] = M_INITIAL_POSITION;
        }else{
            double xPos = outMPos[index - 1].getX() + mpsSpeed[index - 1].getX() * SEC_INTERVAL + 1.0 / 2 * (mpspsEAccel[index - 1].getX() - MPSPS_E_GRAVITY.getX()) * SEC_INTERVAL * SEC_INTERVAL;
            double yPos = outMPos[index - 1].getY() + mpsSpeed[index - 1].getY() * SEC_INTERVAL + 1.0 / 2 * (mpspsEAccel[index - 1].getY() - MPSPS_E_GRAVITY.getY()) * SEC_INTERVAL * SEC_INTERVAL;
            double zPos = outMPos[index - 1].getZ() + mpsSpeed[index - 1].getZ() * SEC_INTERVAL + 1.0 / 2 * (mpspsEAccel[index - 1].getZ() - MPSPS_E_GRAVITY.getZ()) * SEC_INTERVAL * SEC_INTERVAL;

            outMPos[index] = new Quaternion(0, xPos, yPos, zPos);    
        }
    }
}
