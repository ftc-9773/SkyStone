package org.firstinspires.ftc.teamcode.Utilities;

import android.util.Log;

public class PIDController {
    final static String TAG = "PID";
    private double KP;
    private double KI;
    private double KD;
    private double integral;
    private double derivative;
    public double prevError;
    private double output;
    private long lastTime;
    private long deltaTime;
    private boolean firstRun = true;


    private static int maxDeltaTime = 300;
    private static boolean DEBUG = false;

    public PIDController(String jsonfile, String pdescriptor){
        SafeJsonReader json = new SafeJsonReader(jsonfile);
        this.KP = json.getDouble(pdescriptor + "Kp");
        this.KI = json.getDouble(pdescriptor + "Ki");
        this.KD = json.getDouble(pdescriptor + "Kd");
        if (DEBUG) {
            Log.i(TAG, "KP: " + KP);
            Log.i(TAG, "KI: " + KI);
            Log.i(TAG, "KD: " + KD);
        }
    }


    public PIDController( double KP, double KI, double KD) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        if (DEBUG) {
            Log.i(TAG, "KP: " + KP);
            Log.i(TAG, "KI: " + KI);
            Log.i(TAG, "KD: " + KD);
        }
    }

    public double getPIDCorrection(double error) {
        // calculate helper variables
        deltaTime = System.currentTimeMillis() - lastTime;

        // If it is the first run, just return proportional error as i and d cannot be cauculated yet
        if (firstRun || deltaTime > maxDeltaTime) {
            firstRun = false;
            return error  * KP;
        } else {
            // Calculate I and D errors
            integral = integral + (error * deltaTime);
            derivative = (error - prevError) / deltaTime;
            if (DEBUG) {
                Log.i(TAG, "I : "+ integral);
                Log.i(TAG, "D : "+ derivative );
            }
            output = KP * error + KI * integral + KD * derivative;
        }
        // Set previous values for next time

        prevError = error;
        lastTime = System.currentTimeMillis();

        return output;
    }

    public double getPIDCorrection(double target, double actual) {
        return getPIDCorrection(target - actual);
    }
    public void resetPID(){
        this.prevError = 0;
        this.lastTime = System.currentTimeMillis() - maxDeltaTime;
    }
}
