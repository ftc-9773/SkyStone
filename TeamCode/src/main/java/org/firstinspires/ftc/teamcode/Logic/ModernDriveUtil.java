package org.firstinspires.ftc.teamcode.Logic;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Controllers.PIDController;

/**
 * Class containing driving methods for 2019-2020.
 *
 *
 * */
public class ModernDriveUtil {
    Robot robot;
    MecanumDrivebase drivebase = robot.drivebase;
    final static String TAG = "mDriveUtil";
    LinearOpMode opMode;

    //Straight Motion Profile driving helper variables:
    final static double km = (4.2 / 40) / 8.5; //Proportionality constant (torque)
    final static double ke = 12 / (150 * 40); // Proportionality constant (emf)
    final static double rw = 0.0064; // Wheel radius
    final static double m  = 18.688; //Robot mass (kg)
    final static double tf = (4.2 / 40) * 0.2 * 4 / 8.5; //Friction torque in motor (stall torque * no load current * num motor / stall current)
    final static double OMEGA = 0.1; //The big omega (motor resistance + battery resistance)
    double omega = 0; // The small omega (motor rotational speed)
    double v = 0; //
    final double a = (0.9) / 39.37 / 4 * 560; // meters / s^2 in paranthesis, change that
    double s;

    //Straight Motion Profiling config variables. Set via JSON
    double minDistPow, minExitDist;
    double distTol;

    //Some PIDs
    static double[] headingPidCoeffs = new double[3];
    PIDController headingPid;


    //TODO Initialise everything from JSON, when there are config values.
    public ModernDriveUtil(Robot r, LinearOpMode opMode){
        robot = r;
        this.opMode = opMode;
    }

    // Strafe Motion Profiling
    public void strafeStraight(double dist){
        double direction = -90; //Whether to go left or right
        Log.d(TAG, "Strafing with MP " + dist);
        double distSign = Math.signum(dist);
        //direction *= distSign;
        dist = dist * distSign;
        drivebase.stop();
        drivebase.update();
        s = 0;
        double pow;
        double sign;
        double accelerating = 1;
        v = minDistPow  * distSign;
        long[] inits = drivebase.getMotorPositions();
        inits[1] *= -1;
        inits[2] *= -1;
        Log.d(TAG, "Got inits " + inits);
        driveHoldHeading(v, direction, robot.getHeading());
        drivebase.update();
        while((dist - s) > distTol &&  !opMode.isStopRequested()){
            Log.d(TAG, "dsError:" + (dist - s));
            Log.d(TAG, "Velocity: " + v);
            if (dist - s > dist / 2){
                v = (2 * a * s);
                accelerating = 1;
            }else {
                v = (2 * a * (dist - s));
                accelerating = -1;
            }
            sign = Math.signum(v);
            Log.d(TAG, "V^2: " + v);
            v = Math.sqrt(v);
            omega = sign * v * (1 / 560) * 60; //Magic equation
            pow =  (accelerating * a * m * rw * OMEGA / km + OMEGA * ke + tf * omega / km) / 12.7; // More magical equations
            s = Math.abs(avgDistElapsedInchesStrafe(inits));
            pow = Math.max(minDistPow, pow);
            if (sign == 1){
                pow = distSign * Math.min(1, pow);
            } else {
                pow = distSign * Math.max(-1, pow);
            }
            driveHoldHeading(pow, direction, robot.getHeading());
            Log.d(TAG, "Wrote power " + pow);
        }
        driveHoldHeading(0, direction, robot.getHeading());
        drivebase.stop();
        drivebase.update();
    }

    public void driveStraight(double dist){
        Log.d(TAG, "Driving with MP " + dist);
        double distSign = Math.signum(dist);
        dist = dist * distSign;
        drivebase.stop();
        drivebase.update();
        s = 0;
        double pow;
        double sign;
        double accelerating = 1;
        v = minDistPow;
        long[] inits = drivebase.getMotorPositions();
        Log.d(TAG, "Got inits " + inits);
        driveHoldHeading(v, 0, robot.getHeading());
        while((dist - s) > distTol &&  !opMode.isStopRequested()){
            Log.d(TAG, "dsError:" + (dist - s));
            Log.d(TAG, "Velocity: " + v);
            if (dist - s > dist / 2){
                v = (2 * a * s);
                accelerating = 1;
            }else {
                v = (2 * a * (dist - s));
                accelerating = -1;
            }
            sign = Math.signum(v);
            Log.d(TAG, "V^2: " + v);
            v = Math.sqrt(v);
            omega = sign * v * (1 / 560) * 60; //Magic equation
            pow =  (accelerating * a * m * rw * OMEGA / km + OMEGA * ke + tf * omega / km) / 12.7; // More magical equations
            s = Math.abs(avgDistElapsedInchesForward(inits));
            pow = Math.max(minDistPow, pow);;
            if (sign == 1){
                pow = distSign * Math.min(1, pow);
            } else {
                pow = distSign * Math.max(-1, pow);
            }
            driveHoldHeading(pow, 0, robot.getHeading());
            Log.d(TAG, "Wrote power " + pow);
        }
        driveHoldHeading(0, 0, robot.getHeading());
        drivebase.stop();
        drivebase.update();
    }

    /**
     * a semi-internal funciton used to keep the robot pointed in a certian direction while driving.
     * @param magnitude driving function magnitude in motor power (i.e. on range -1.0 to 0.0 to 1.0)
     * @param angle the angle at which the robot is desired to drive in radians.
     * @param heading the heading that the robot is desired to face.
     */
    public void driveHoldHeading (double magnitude, double angle, double heading){
        double currHeading = robot.getHeading();
        double error = heading - currHeading;
        Log.d(TAG, "Error: " + error);
        double correction = headingPid.getPIDCorrection(error);

        angle = Math.toRadians(angle + 90);
        //angle = currHeading - angle;

        double x = Math.cos(angle)*magnitude;
        double y = Math.sin(angle)*magnitude;

        Log.d(TAG, "Wrote to drive. correction was" + correction +" x was " + x + " y was " + y);

        drivebase.drive(x, y, -correction, false);
        drivebase.update();
    }

    /**
     * Get the distance traveled sideways from the initial positions.
     * TODO Replace this and avgElapsedInchesForward with one function.
     * */
    public double avgDistElapsedInchesStrafe(long[] initpositions){
        long[] positions = drivebase.getMotorPositions();
        positions[1] *= -1;
        positions[2] *= -1;
        double sum=0;
        for(int i = 0; i<4; i++){
            double diff = (double) positions[i] - initpositions[i];
            //Log.d(TAG, "adding motor position: " +i+" ticks: " + positions[i]  + " diff:" + diff);
            if(i==0 ||i==2) diff *=-1;
            sum +=diff;
        }
        sum /=4;
        sum /= drivebase.COUNTS_PER_INCH;;
        Log.d(TAG, "STRAFE INCHES " + sum);
        return sum ;
    }
    /**
     * Get the distance traveled sideways from the initial positions.
     * TODO Replace this and avgElapsedInchesStrafe with one function.
     * */
    public double avgDistElapsedInchesForward(long[] initpositions) {
        long[] positions = drivebase.getMotorPositions();
        double sum = 0;
        for (int i = 0; i < 4; i++) {
            double diff = (double) positions[i] - initpositions[i];
            //Log.d(TAG, "adding motor position: " +i+" ticks: " + positions[i]  + " diff:" + diff);
            if (i == 0 || i == 2) diff *= -1;
            sum += diff;
        }
        sum /= 4;
        sum /= drivebase.COUNTS_PER_INCH;
        ;
        Log.d(TAG, "INCHES " + sum);
        return sum;
    }
}

