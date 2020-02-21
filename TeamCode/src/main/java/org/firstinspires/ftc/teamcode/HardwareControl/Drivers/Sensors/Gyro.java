package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

/**
 * Class for interacting with the imu built into the RevHub
 * */
public class Gyro {

    BNO055IMU imuLeft;
    //BNO055IMU imuRight;

    private double zeroPosition = 0;
    //private SafeJsonReader jsonZero;

    //private double zeroPositionRight = 0;

    private static String TAG = "ftc9773_Gyro";
    private static boolean DEBUG = false;


    private double lastImuAngle;
    private double lastImuVelocity = 0;

    private long lastReadTime = -1;
    private static final int minReadDeltaTime = 40;


    // Init
    public Gyro (HardwareMap hardwareMap) {
        ///// Initialize the IMU /////
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imuLeft                         = hardwareMap.get(BNO055IMU.class, "imu");

        imuLeft.initialize(parameters);
        if (DEBUG) Log.d(TAG, imuLeft.toString());
        assert (imuLeft !=null);
        readImu();
        readImu();
    }

    // For directly reading the IMU's values
    private double getImuAngle() { return imuLeft.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;  }
    private double getImuAngularVelocity() { return imuLeft.getAngularVelocity().zRotationRate; }

    // Reads the IMU and updates 3rd degree taylor series modeling
    private void readImu() {

        // Read Angle
        lastImuAngle = getImuAngle();

        // Save time read
        lastReadTime = System.currentTimeMillis();

        // Read Velocity and save old reading
        lastImuVelocity = getImuAngularVelocity();
    }

    // Returns calculated orientation
    public double getHeading(boolean forceNewReading) {

        //Check if new reading is required
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastReadTime > minReadDeltaTime || forceNewReading)  readImu();

        // Calculate estimated position
//        long dt = currentTime - lastReadTime;
//        return -(lastImuAngle + lastImuVelocity * dt) - zeroPosition;
        return lastImuAngle - zeroPosition;
    }

    // Returns calculated velocity
    public double getVelocity() {
        // Check if new reading is required
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastReadTime > minReadDeltaTime)  readImu();

        // Calculate estimated velocity
        long dt = currentTime - lastReadTime;
        return -(lastImuVelocity);
    }

    // Alias
    public double getHeading() { return getHeading(false); }
    public void setZeroPosition() {
        zeroPosition = getHeading(true);
    }

    public boolean  isUpdated(){
        return System.currentTimeMillis() - lastReadTime > minReadDeltaTime;
    }
    public void close(){
        imuLeft.close();
    }
/*
    public void recordHeading() {
        jsonZero.modifyDouble("currentAngle", getHeading());
        jsonZero.modifyString("writeTime", "" + System.currentTimeMillis());
        jsonZero.updateFile();
        Log.i("GyroPositionLogged", "Position: " +  jsonZero.getDouble("currentAngle"));
        Log.i("GyroPositionLogged", "Time: " + jsonZero.getInt("writeTime"));
    }
*/
}
