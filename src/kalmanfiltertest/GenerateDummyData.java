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
import java.util.Random;


public class GenerateDummyData {
    int samplesLength;
    double secInterval;
    Quaternion utECmpsBase;
    Quaternion mpspsEGravity;
    
    Quaternion initialAttitude;
    Quaternion mpsInitialSpeed;
    Quaternion mInitialPosition;
    
    //output format
    //1:time, 2:heading, 3:pitch, 4:role, 
    //5-7:accel[3], 8-10:gyro[3], 11-13:cmps[3], 
    //14-16:speed[3], 17:19:pos[3]

//    public GenerateDummyData(int sampleLength, double sampleLength,Quaternion initialAttitude) {
//    }
    public GenerateDummyData(int sampleLength,double secInterval, Quaternion utECmpsBase,Quaternion mpspsEGravity,
            Quaternion initialAttitude,Quaternion mpsInitialSpeed,Quaternion mInitialPosition){
        this.samplesLength = sampleLength;
        this.secInterval = secInterval;
        this.utECmpsBase = utECmpsBase;
        this.mpspsEGravity = mpspsEGravity;
        this.initialAttitude = initialAttitude;
        this.mpsInitialSpeed = mpsInitialSpeed;
        this.mInitialPosition = mInitialPosition;
    }
    
    public void generate(Quaternion[] mpspsBAccel,Quaternion[] rpsBGyro,Quaternion[] attitude,Quaternion[] mpspsEAccel,
            Quaternion[] mpsESpeed,Quaternion[] mEPos,Quaternion[] utBCmps){
        
        
        initializeGyro(rpsBGyro);
        initializeAccel(mpspsBAccel);

        
        calcAttitude(attitude, rpsBGyro);
        calcEAccel(mpspsEAccel, attitude, mpspsBAccel);
        calcSpeed(mpsESpeed, mpspsEAccel);
        calcPos(mEPos, mpsESpeed,mpspsEAccel);
        calcCmps(utBCmps, attitude);        
    }
    
    private void initializeGyro(Quaternion[] rpsBGyro){
        for (int i = 0; i < rpsBGyro.length; i++) {
            double wX = 0.0;
            double wY = 0.0;
            double wZ = 2*Math.PI/10;
            
            rpsBGyro[i] = new Quaternion(0,wX,wY,wZ);
        }
    }
    private void initializeAccel(Quaternion[] mpspsBAccel){
        for (int i = 0; i < mpspsBAccel.length; i++) {
            double aclX = 0.0;
            double aclY = 2*Math.PI;
            double aclZ = -9.8;
            
            mpspsBAccel[i] = new Quaternion(0,aclX,aclY,aclZ);
        }
    }
    
    
    private void  calcAttitude(Quaternion[] outAttitude,Quaternion[] rpsBGyro){
        outAttitude[0] = initialAttitude;
        for (int i = 1; i < outAttitude.length; i++) {
            double dx = Math.sin(rpsBGyro[i-1].getX()*secInterval/2);
            double dy = Math.sin(rpsBGyro[i-1].getY()*secInterval/2);
            double dz = Math.sin(rpsBGyro[i-1].getZ()*secInterval/2);
            double dw = Math.sqrt(1 - (dx*dx + dy*dy + dz*dz));
            
            Quaternion roter = new Quaternion(dw, dx, dy, dz);
            
            outAttitude[i] = roter.mul(outAttitude[i-1]);
            outAttitude[i] = outAttitude[i].normalize();
        }
    }
    private void calcEAccel(Quaternion[] outMpspsEAccel, Quaternion[] attitude, Quaternion[] mpspsBAccel){
        for (int i = 0; i < outMpspsEAccel.length; i++) {
            outMpspsEAccel[i] = attitude[i].mul(mpspsBAccel[i]).mul(attitude[i].con());
        }
    }
    
    private void calcSpeed(Quaternion[] outMpsSpeed, Quaternion[] mpspsEAccel){
        outMpsSpeed[0] = mpsInitialSpeed;
        for (int i = 1; i < outMpsSpeed.length; i++) {
            double xSpeed = outMpsSpeed[i-1].getX() + ((mpspsEAccel[i-1].getX()+mpspsEAccel[i].getX())/2-mpspsEGravity.getX())*secInterval;
            double ySpeed = outMpsSpeed[i-1].getY() + ((mpspsEAccel[i-1].getY()+mpspsEAccel[i].getY())/2-mpspsEGravity.getY())*secInterval;
            double zSpeed = outMpsSpeed[i-1].getZ() + ((mpspsEAccel[i-1].getZ()+mpspsEAccel[i].getZ())/2-mpspsEGravity.getZ())*secInterval;
            
            outMpsSpeed[i] = new Quaternion(0, xSpeed, ySpeed, zSpeed);
        }
    }
    
    private void calcPos(Quaternion[] outMPos, Quaternion[] mpsSpeed, Quaternion[] mpspsEAccel){
        outMPos[0] = mInitialPosition;
        for (int i = 1; i < outMPos.length; i++) {
            double xPos = outMPos[i-1].getX() + mpsSpeed[i-1].getX()*secInterval + 1.0/2*(mpspsEAccel[i-1].getX()-mpspsEGravity.getX())*secInterval*secInterval;
            double yPos = outMPos[i-1].getY() + mpsSpeed[i-1].getY()*secInterval + 1.0/2*(mpspsEAccel[i-1].getY()-mpspsEGravity.getY())*secInterval*secInterval;
            double zPos = outMPos[i-1].getZ() + mpsSpeed[i-1].getZ()*secInterval + 1.0/2*(mpspsEAccel[i-1].getZ()-mpspsEGravity.getZ())*secInterval*secInterval;
            
            outMPos[i] = new Quaternion(0, xPos, yPos, zPos);
        }
    }
    private void calcCmps(Quaternion[] outBUtCmps,Quaternion[] attitude){
        for (int i = 0; i < outBUtCmps.length; i++) {
            outBUtCmps[i] = attitude[i].con().mul(utECmpsBase).mul(attitude[i]);
        }
    }
}
