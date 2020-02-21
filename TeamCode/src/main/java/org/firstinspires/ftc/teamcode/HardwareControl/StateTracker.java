package org.firstinspires.ftc.teamcode.HardwareControl;


import Jama.Matrix;

public class StateTracker {
    double Vx = 0, Vy = 0; //Horizontal and vertical velocity in m/s
    double Sx = 0, Sy = 0; //Position in m
    double theta = 0, omega = 0;//Heading (radians) and rotational velocity (rad/s).

    double last_Sx = 0;
    double last_Sy = 0;
    double last_T  = 0;

    long lastVxReadingTime;
    long lastVyReadingTime;
    long lastWReadingTime;

    public StateTracker(){
        lastVxReadingTime = System.currentTimeMillis();
        lastVxReadingTime = System.currentTimeMillis();
        lastWReadingTime  = System.currentTimeMillis();
    }

    public void registerdx(double dx){
        setSx(Sx + dx);
    }
    public void registerdy(double dy){
        setSy(Sy + dy);
    }
    public void registerdT(double dT){
        setTheta(theta + dT);
    }

    public void setVx(double vx) {
        Vx = vx;
    }

    public void setVy(double vy) {
        Vy = vy;
    }

    public void setSx(double sx) {
        last_Sx = Sx;
        Sx = sx;
        setVx((Sx - last_Sx) / System.currentTimeMillis() - lastVxReadingTime);
        lastVxReadingTime = System.currentTimeMillis();
    }

    public void setSy(double sy) {
        last_Sy = Sy;
        Sy = sy;
        setVy((Sy - last_Sy) / System.currentTimeMillis() - lastVyReadingTime);
        lastVyReadingTime = System.currentTimeMillis();
    }

    public void setTheta(double theta) {
        last_T = this.theta;
        this.theta = theta;
        setVx((this.theta - last_T) / System.currentTimeMillis() - lastWReadingTime);
        lastWReadingTime = System.currentTimeMillis();
    }

    public void setOmega(double omega) {
        this.omega = omega;
    }

    public Matrix getState(){
        double[][] state = {{Sx, Sy, Vx, Vy, omega, theta}};
        return new Matrix(state);
    }
}
