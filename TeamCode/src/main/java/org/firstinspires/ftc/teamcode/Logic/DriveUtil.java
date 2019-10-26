package org.firstinspires.ftc.teamcode.Logic;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.VoltSensor;
import org.firstinspires.ftc.teamcode.HardwareControl.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Controllers.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;
import org.firstinspires.ftc.teamcode.Utilities.misc.Timer;

import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

/**
 * The Motion Profiling drive util class creates and holds various methods for driving
 * This program is made to be able to be used in various circumstances,
 * <p>
 * This program is made in the paradigm followed by the rest of the robot drivers. Between each loop,
 * it calls the robot.update(); method, allowing other processes to happen simultaneously.
 *
 * @author  Zachary Eichenberger, Cadence Weddle -ftc robocracy 9773
 * @version 1.1
 * */
public class DriveUtil {

    static final String TAG = "ftc9773_drivePID";
    static final boolean scalingClip = false;
    //operator things;
    Robot robot;
    LinearOpMode opMode;
    MecanumDrivebase drivebase;
    Gyro gyro;

    // information
    double ticksPerInch;

    // Json
    SafeJsonReader json;

    // pid coeffs
    final double minDistPow, minExitDist;
    final double maxTurnPower;
    double distTol;
    static double[] rotPidCoeffs = new double[3];
    double rotTol, rotExitSpeed;
    double rotMinPow;
    PIDController rotPid;
    static double[] headingPidCoeffs = new double[3];
    PIDController headingPid;

    final static double km = (4.2 / 40) / 8.5; //Proportionality constant (torque)
    final static double ke = 12 / (150 * 40); // Proportionality constant (emf)
    final static double rw = 0.0064; // Wheel radius (m)
    final static double m  = 18.688; //Robot mass (kg)
    final static double tf = (4.2 / 40) * 0.2 * 4 / 8.5; //Friction torque in motor (stall torque * no load current * num motor / stall current)
    final static double OMEGA = 0.1; //The big omega (motor resistance + battery resistance)
    double omega = 0; // The small omega (motor rotational speed)
    double v = 0; //
    final double a = (0.9) / 39.37 / 4 * 560; // meters / s^2 in paranthesis  "????????????" —Future (now past) Cadence
    double s;

    /**
     * Constructor for the PID drive Util class.
     * initializes all of the parts, and reads values from JSON
     *
     * takes the robot and linear opMode classes, used to initialize and eventuallly update stuff
     * @param robot
     * @param opMode
     */
    public DriveUtil(Robot robot, LinearOpMode opMode){
        this.robot = robot;
        this.opMode = opMode;
        drivebase = robot.drivebase;
        drivebase.runWithEncoders();
        gyro = robot.gyro;
        ticksPerInch = drivebase.COUNTS_PER_INCH;

        json = new SafeJsonReader("DrivePidValues");

        distTol = json.getDouble("distTol", 0.5);
        minDistPow = json.getDouble("minDistPow", 0.02);
        minExitDist = json.getDouble("minExitDist", 0.02);
        maxTurnPower = json.getDouble("maxTurnPow", 0.6);

        //  rotPid
        rotPidCoeffs[0] = json.getDouble("rotKp",0.11);
        rotPidCoeffs[1] = json.getDouble("rotKi", 0);
        rotPidCoeffs[2] = json.getDouble("rotKd", 0.15);
        rotPid = new PIDController(rotPidCoeffs[0],rotPidCoeffs[1], rotPidCoeffs[2]);
        Log.d(TAG, "created rotation PID controller pid coeff array: " + Arrays.toString(rotPidCoeffs));

        rotTol = json.getDouble("rotTol", 0.0174533);// currently 1 degree
        rotExitSpeed = json.getDouble("rotExitSpeed", 0.2); // currently 5º/sec
        rotMinPow = json.getDouble("rotMinSpeed", 0.14);
        //  rotPid
        headingPidCoeffs[0] = json.getDouble("headingKp",0.1);
        headingPidCoeffs[1] = json.getDouble("headingKi", 0);
        headingPidCoeffs[2] = json.getDouble("headingKd", 0);
        headingPid = new PIDController(headingPidCoeffs[0],headingPidCoeffs[1], headingPidCoeffs[2]);
        Log.d(TAG, "created heading PID controller pid coeff array: " + Arrays.toString(headingPidCoeffs));

        drivebase.runWithoutEncoders();

    }

    //General motion profile drive
    public void drive(double dist, double angle){
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
            s = Math.abs(avgDistElapsed(inits));
            pow = Math.max(minDistPow, pow);;
            if (sign == 1){
                pow = distSign * Math.min(1, pow);
            } else {
                pow = distSign * Math.max(-1, pow);
            }
            driveHoldHeading(pow, angle, robot.getHeading());
            Log.d(TAG, "Wrote power " + pow);
        }
        driveHoldHeading(0, 0, robot.getHeading());
        drivebase.stop();
        drivebase.update();
    }


    /**
     * a semi-internal function used to keep the robot pointed in a certian direction while driving.
     * @param magnitude driving function magnitude in motor power (i.e. on range -1.0 to 0.0 to 1.0)
     * @param angle the angle at which the robot is desired to drive in radians.
     * @param heading the heading that the robot is desired to face.
     */
     public void driveHoldHeading (double magnitude, double angle, double heading){
         double currHeading = gyro.getHeading();
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
     * A function that turns the robot to a certain field centric position
     *  runs a loop while it can, and exits once at the correct position
     * @param goalHeading the intended feild centric heading in degrees.
     */
     public void turnToAngle(double goalHeading) {
         Log.d(TAG,"startingTurnToAnlge: " + goalHeading);

         final double targetAngleRad = Math.toRadians(goalHeading);

         // For calculating rotational speed:
         double lastHeading;
         double currentHeading = gyro.getHeading();
         Log.d(TAG,"startingHeading: " + currentHeading );


         double lastTime;
         double currentTime = System.currentTimeMillis();
         double lastError =0.0;

         // For turning PID
         double error;

         boolean firstTime = true;
         while (!opMode.isStopRequested()) {

             // update time and headings:
             currentHeading = gyro.getHeading();

             lastTime = currentTime;
             currentTime = System.currentTimeMillis();

             error = setOnNegToPosPi(targetAngleRad - currentHeading);
             double rotation = rotPid.getPIDCorrection(error);

             // may add this in if dt is too weak
             if (rotation > 0 && rotation < rotMinPow) {
                 rotation = rotMinPow;
             } else if (rotation < 0 && Math.abs(rotation) < rotMinPow) {
                 rotation = -rotMinPow;
             }

             if (rotation > maxTurnPower)
                 rotation = maxTurnPower;
             else if (rotation < -maxTurnPower)
                 rotation = -maxTurnPower;


             Log.d(TAG,"writingToDrive: Error: "+ error + " Correction: " + rotation );
             Log.d(TAG, "error in degrees: "+ Math.toDegrees(error));
             drivebase.drive(0.0, 0, -rotation, false);
             drivebase.update();



             // Check to see if it's time to exit
             // Calculate speed
             double speed;
             if (currentTime == lastTime || firstTime) {
                 speed = 0.003;
             } else {
                 speed = Math.abs(error - lastError) / (currentTime - lastTime);
             }
             lastError = error;

             if ( Math.abs(error) < Math.abs(rotTol) && speed < rotExitSpeed) {
                 Log.i(TAG, "ending rotation, should be at heading");
                 break;
             }
             firstTime = false;
             // update robot
             robot.update();
             // temporary
         }
         drivebase.stop();
        rotPid.resetPID();
     }

    private double setOnNegToPosPi (double num) {
        while (num > Math.PI) {
            num -= 2*Math.PI;
        }
        while (num < -Math.PI) {
            num += 2*Math.PI;
        }
        return num;
    }
    /**
     * Get average distance traveled
     * */
    public double avgDistElapsed(long[] initpositions){
        long[] last = initpositions;
        long[] cur = drivebase.getMotorPositions();


        double dx;
        dx = ((cur[0] - last[0]) + (cur[1] - last[1])) / 2;
        dx /= MecanumDrivebase.COUNTS_PER_INCH;
        double dy;
        dy = ((cur[0] - last[0]) - (cur[1] - last[1])) / 2;
        dy /= MecanumDrivebase.COUNTS_PER_INCH;
        return Math.sqrt(dy * dy + dx * dx);
    }

    public void align(){
        double tol = 0.2;
        double[] readings = robot.getDistSensorReadings();
        while (Math.abs(-readings[0] + readings[1]) > tol){
            turnToAngle(robot.heading + Math.toDegrees(Math.atan(readings[1] - readings[0]) / 2));
            readings = robot.getDistSensorReadings();
        }
    }

}
