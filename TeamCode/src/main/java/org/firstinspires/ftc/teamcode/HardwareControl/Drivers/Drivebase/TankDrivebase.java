package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

public class TankDrivebase {

    static private final double     COUNTS_PER_MOTOR_REV    = 560;    //
    static private final double     WHEEL_TURNS_PER_MOTOR_REV = 30.0 / 38.0;
    static private final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static private final double     ROBOT_DIAMETER_INCHES   = 7.322 * 2;
    public static final double      COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV*WHEEL_TURNS_PER_MOTOR_REV ) / (WHEEL_DIAMETER_INCHES * Math.PI);

    final public String TAG = "TankDrivebase";

    //High precision mode
    double slowFactor = 0.3;

    public DcMotor[] driveMotors = new DcMotor[4];

    private double[] motorPowers = new double[4];
    private SafeJsonReader reader;

    public TankDrivebase(HardwareMap hwMap){
        //this.reader = new SafeJsonReader("TankDrivebase");
        //slowFactor = reader.getDouble("slowFactor", 0.3);
        driveMotors[0] = hwMap.get(DcMotor.class, "fldrive");
        driveMotors[1] = hwMap.get(DcMotor.class, "frdrive");
        driveMotors[2] = hwMap.get(DcMotor.class, "bldrive");
        driveMotors[3] = hwMap.get(DcMotor.class, "brdrive");

        for (DcMotor motor: driveMotors){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        runWithEncoders();
    }

    public void runWithEncoders(){for(DcMotor d:driveMotors){ d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}}

    //Scale value to respond more at high values.
    private double scale(double val) {
        if (Math.abs(val) > 0.05) {
            return Math.pow(val, 3);
        } else {
            return 0;
        }
    }

    public void update() {
        for (int i = 0; i<4; i++) {
            driveMotors[i].setPower(motorPowers[i]);
            /*Log.d("Drivebase", "Wrote power " + motorPowers[i] + " to motor " + i); */
        }
    }

    public void stop() {
        for (int i=0; i<4; i++) {motorPowers[i] = 0;}
        update();
    }

    //Note, positive turn is right
    public void driveParam(double pow, double turn){
        driveHuman(pow + turn, pow - turn, false, false);
    }

    public void driveHuman(double l, double r, boolean slow, boolean scale){
        if (slow){
            l *= slowFactor;
            r *= slowFactor;
        }
        if (scale){
            l = scale(l);
            r = scale(r);
        }
        motorPowers[0] = l;
        motorPowers[1] = -r;
        motorPowers[2] = l;
        motorPowers[3] = -r;
    }

    public double[] getEncoderPositions(){
        double[] output = new double[4];
        output[0] = driveMotors[0].getCurrentPosition();
        output[1] = driveMotors[1].getCurrentPosition();
        output[2] = driveMotors[2].getCurrentPosition();
        output[3] = driveMotors[3].getCurrentPosition();

        return output;
    }

}
