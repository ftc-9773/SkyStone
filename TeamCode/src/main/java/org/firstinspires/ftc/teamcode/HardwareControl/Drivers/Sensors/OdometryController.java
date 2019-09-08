package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Logic.Geometry.Vector;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

public class OdometryController {
    public final double INCH_PER_TICK = 0.01835868 / 4;


    // Encoders
    private DcMotor encoder1;
    private DcMotor encoder2;
    private DcMotor encoder3;

    //Last distance calculated
    private double lastEncoderPos1 = 0;
    private double lastEncoderPos2 = 0;
    private double lastEncoderPos3 = 0;


    // time
    private long lastTime;

    // Last Position
    private double lastX;
    private double lastY;
    private double lastAng;

    // Last Velocity
    private double lastXVel;
    private double lastYVel;
    private double lastAngVel;

    // Last Acceleration
    private double lastXAcel;
    private double lastYAcel;
    private double lastAngAcel;

    // Current Position
    public double curX;
    public double curY;
    public double curAng;

    // Current Velocity
    public double curXVel;
    public double curYVel;
    public double curAngVel;

    // Current Acceleration
    public double curXAcel;
    public double curYAcel;
    public double curAngAcel;

    // Current Jerk
    public double curXJerk;
    public double curYJerk;
    public double curAngJerk;

    private static double[][] inverseMatrix =  {{-2.27864583e-01,  1, -2.27864583e-01},
            {0.5, 0, -0.5},
            {6.51041667e-02, 0,  6.51041667e-02}};

    SafeJsonReader InvMatrixDict;

    public void readInvMatrix() {
        /*
        inverseMatrix[0][0] = InvMatrixDict.getDouble("k00");
        inverseMatrix[0][1] = InvMatrixDict.getDouble("k01");
        inverseMatrix[0][2] = InvMatrixDict.getDouble("k02");

        inverseMatrix[1][0] = InvMatrixDict.getDouble("k10");
        inverseMatrix[1][1] = InvMatrixDict.getDouble("k11");
        inverseMatrix[1][2] = InvMatrixDict.getDouble("k12");

        inverseMatrix[2][0] = InvMatrixDict.getDouble("k20");
        inverseMatrix[2][1] = InvMatrixDict.getDouble("k21");
        inverseMatrix[2][2] = InvMatrixDict.getDouble("k22");
        */
    }

    public OdometryController(HardwareMap hwMap, double X, double Y, double Ang) {
        encoder1 = hwMap.dcMotor.get("leftEncoder");
        encoder2 = hwMap.dcMotor.get("centerEncoder");
        encoder3 = hwMap.dcMotor.get("rightEncoder");

        InvMatrixDict = new SafeJsonReader("odometryInvMatrix");

        readInvMatrix();

        this.curX = X;
        this.curY = Y;
        this.curAng = Ang;
    }
    public OdometryController(HardwareMap hwMap) {
        this(hwMap, 0, 0, 0);
    }


    public void updatePose(){
        //Distance from the last time calculated
        double curPos1 = encoder1.getCurrentPosition() * INCH_PER_TICK;
        double curPos2 = encoder2.getCurrentPosition() * INCH_PER_TICK;
        double curPos3 = encoder3.getCurrentPosition() * INCH_PER_TICK;

        double d1 = curPos1 - lastEncoderPos1;
        double d2 = curPos2 - lastEncoderPos2;
        double d3 = curPos3 - lastEncoderPos3;

        // Updating X, Y, and Ang
        double deltaX = inverseMatrix[0][0] * d1 + inverseMatrix[0][1] * d2 + inverseMatrix[0][2] * d3;
        double deltaY = inverseMatrix[1][0] * d1 + inverseMatrix[1][1] * d2 + inverseMatrix[1][2] * d3;
        curAng += inverseMatrix[2][0] * d1 + inverseMatrix[2][1] * d2 + inverseMatrix[2][2] * d3;

        Vector travelVector = new Vector(true, deltaX, deltaY);
        travelVector.rotateVector(-curAng);
        curX += travelVector.getX();
        curY += travelVector.getY();

        // Current Velocity, Acceleration, Jerk

        long curTime = System.currentTimeMillis();
        int dt = (int)(curTime - lastTime);

        // Velocity
        curXVel = (curX - lastX) / dt;
        curYVel = (curY - lastY) / dt;
        curAngVel = (curAng - lastAng) / dt;


        // Acceleration
        curXAcel = (curXAcel - lastXVel) / dt;
        curYAcel = (curYAcel - lastYVel) / dt;
        curAngAcel = (curAngAcel - lastAngVel) / dt;

        // Jerk
        curXJerk = (curXAcel - lastXAcel) / dt;
        curYJerk = (curYAcel - lastYAcel) / dt;
        curAngJerk = (curAngAcel - lastAngAcel) / dt;


        //Updating variables
        lastX = curX;
        lastY = curY;
        lastAng = curAng;

        lastEncoderPos1 = curPos1;
        lastEncoderPos2 = curPos2;
        lastEncoderPos3 = curPos3;

        lastXVel = curXVel;
        lastYVel = curYVel;
        lastAngVel = curAngVel;

        lastXAcel = curXAcel;
        lastYAcel = curYAcel;
        lastAngAcel = curAngAcel;

        lastTime = curTime;
    }

    public void setPosition(double x, double y, double ang) {
        this.curX = x;
        this.curY = y;
        this.curAng = ang;
    }

    public double[] getPosition(){
        updatePose();
        double[] out = {curX, curY, curAng};
        return out;
    }
}
