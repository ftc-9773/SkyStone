package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Logic.Geometry.Vector;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

public class MecanumDrivebase {

    static private final double     COUNTS_PER_MOTOR_REV    = 560;    //
    static private final double     WHEEL_TURNS_PER_MOTOR_REV = 30.0 / 38.0;
    static private final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static private final double     ROBOT_DIAMETER_INCHES   = 7.322 * 2;
    public static final double      COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV*WHEEL_TURNS_PER_MOTOR_REV ) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static private final double DRIVE_SCALING = 3; // Must be odd
    static private final double ROTATION_SCALING = 0.8; // 0 is none
    private static final boolean DEBUG = false;

    // slowdown mode stuff
    public double slowdownScaleFactor = 0.5;
    public boolean slowdownMode = false;

    static final double MAX_TRANSLATIONAL_SPEED = 1.0;
    static final double MAX_ROTATIONAL_SPEED = 1.0;




    public DcMotor[] driveMotors;

    private double[] motorPowers = new double[4];

    Telemetry telemetry;
    SafeJsonReader reader = new SafeJsonReader("DrivePidValues");

    public MecanumDrivebase(HardwareMap hwMap, Telemetry telem) {
        // init wheels
        this.telemetry = telem;
        driveMotors = new DcMotor[4];
        driveMotors[0] = hwMap.get(DcMotor.class, "fldrive");
        driveMotors[1] = hwMap.get(DcMotor.class, "frdrive");
        driveMotors[2] = hwMap.get(DcMotor.class, "bldrive");
        driveMotors[3] = hwMap.get(DcMotor.class, "brdrive");
        //pid coeffs for different motion stuff.




        for (DcMotor motor:driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //motor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(reader.getDouble("kp"), reader.getDouble("ki"), reader.getDouble("kd")));
        }
        runWithEncoders();
    }

    private double scale(double val) {
        if (Math.abs(val) > 0.05) {
            return Math.pow(val, 3);
        } else {
            return 0;
        }
    }

    private double[] scaleDriving(double x, double y, double rotation) {
        Vector vec = new Vector(true, x, y);

        double[] finalSpeeds = new double[3];

        if (Math.abs(vec.getMagnitude()) > 0.05) {
            double mag =   DRIVE_SCALING * scale(vec.getMagnitude());
            finalSpeeds[0] = mag * Math.sin(vec.getAngle());
            finalSpeeds[1] = mag * Math.cos(vec.getAngle());

            finalSpeeds[2] = scale(rotation) * (ROTATION_SCALING * mag + 1);
        } else {
            finalSpeeds[0] = 0;
            finalSpeeds[1] = 0;

            finalSpeeds[2] = scale(rotation);
        }
        return finalSpeeds;
    }

    // Clean Version
    public void drive(double x, double y, double rotation, boolean scale) {

        if (scale)  {
            double[] arr = scaleDriving(x, y, rotation);
            x = arr[0];
            y = arr[1];
            rotation = arr[2];
        }

        runWithEncoders();

        motorPowers[0] = y + x + rotation;
        motorPowers[1] = y - x - rotation;
        motorPowers[2] = y - x + rotation;
        motorPowers[3] = y +  x - rotation;

        motorPowers[0] *= -1;
        motorPowers[2] *= -1;


        double maxVal = Math.max(Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1])),
                        Math.max(Math.abs(motorPowers[2]), Math.abs(motorPowers[3])));

        // Check that values are < 1. Normalize otherwise
        if (maxVal > 1) { for (int i = 0; i < 4; i++) { motorPowers[i] /= maxVal;} }

        telemetry.addData("MotorPowers", String.format("%.1f, %.1f, %.1f, %.1f", motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]));

    }

    public void drivePolar(double mag, double theta, double rotation, boolean scale) {
        double x = Math.sin(theta)*mag;
        double y = Math.cos(theta)*mag;

        drive(x, y, rotation, scale);
    }


    public void update() {
        for (int i = 0; i<4; i++) {
            if(slowdownMode) driveMotors[i].setPower(motorPowers[i]*slowdownScaleFactor);
            else driveMotors[i].setPower(motorPowers[i]);
            if (DEBUG) Log.d("Drivebase", "Wrote power " + motorPowers[i] + " to motor " + i);
        }
    }

    public void stop() {
        for (int i=0; i<4; i++) {motorPowers[i] = 0;}
        update();
    }

    public long[] getMotorPositions(){
        long[] value = new long[4];
        for (int i = 0; i< 4; i++)
            value[i] = driveMotors[i].getCurrentPosition();
        return value;
    }

    public void runWithEncoders(){for(DcMotor d:driveMotors){ d .setMode(DcMotor.RunMode.RUN_USING_ENCODER);}}
    public void runWithoutEncoders() { for (DcMotor d:driveMotors) {d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}}
    public void runToPosition(){for(DcMotor d:driveMotors){
        //d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        d.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }}


    public double[] getPos(){
        double[] pos = new double[4];
        pos[0] = driveMotors[0].getCurrentPosition();
        pos[1] = driveMotors[1].getCurrentPosition();
        pos[2] = driveMotors[2].getCurrentPosition();
        pos[3] = driveMotors[3].getCurrentPosition();
        return pos;
    }

    void setSlowdownScaleFactor (double val ){
        slowdownScaleFactor = val;
    }

    public double getMinPower(LinearOpMode o){
        runWithEncoders();
        long[] init = getMotorPositions();
        telemetry.addData("Init", init);
        telemetry.update();
        double pow = 0.01;
        setMotorPowers(pow);
        double speed = 0;
        double start = System.currentTimeMillis();
        while (speed < 0.1 && !o.isStopRequested()){
            pow += 0.01;
            setMotorPowers(pow);
            speed = minus(getMotorPositions(), init) / (System.currentTimeMillis() - start);
            //start = System.currentTimeMillis();
            if (DEBUG) {
                Log.d("Drivebase", "Speed = " + speed);
                Log.d("Drivebase", "Power = " + pow);
            }
            telemetry.addData("Speed", speed);
            telemetry.addData("SetPower", pow);
            telemetry.update();
            //Wait(1, o);
        }
        setMotorPowers(0);
        return pow;
    }

    private void Wait(double t, LinearOpMode o){
        double start = System.currentTimeMillis();
        while(System.currentTimeMillis() - start < t && !o.isStopRequested()){
            continue;
        }

    }

    private void setMotorPowers(double pow){
        for (int i = 0; i < 4; i++){
            if(slowdownMode) driveMotors[i].setPower(( i % 2 == 0 ? -1:1) * pow * slowdownScaleFactor);
            else driveMotors[i].setPower(( i % 2 == 0 ? -1:1) * pow);
        }
    }

    private double minus(long[] a, long[] y){
        double sum = 0;
        if (a.length != y.length){
            return -1;
        }
        for (int i = 0; i <a.length; i++){
            sum += (a[i] - y[i]) * (a[i] - y[i]);
        }
        return Math.sqrt(sum);
    }

    public void getPowersLogged(Telemetry telemetry){
        if (DEBUG) {
            for(int i = 0; i< 4; i++) {
                //telemetry.addData("Motor power of motor: "+ i, driveMotors[i].getPower());
                Log.d("ftc9773_motorPowers", "motor " + i + driveMotors[i].getPowerFloat());
            }
        }
    }

    public void driveTank(double p1, double p2){
        motorPowers[0] = p1;
        motorPowers[2] = p1;
        motorPowers[1] = p2;
        motorPowers[3] = p2;

    }
}
