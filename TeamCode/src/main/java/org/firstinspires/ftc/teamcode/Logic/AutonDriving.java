package org.firstinspires.ftc.teamcode.Logic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

import Jama.Matrix;

/**
 * Diagram of the mecanum drivebase:
 *
 * \0\___/1/
 *  |     |
 *  |     |
 * /3/___\2\
 * (Angle represents the angle of the mecanum wheel. Note: index should align with that of mecanumdrivebase)
 * */
public class AutonDriving {
    public Matrix K; //4x6 matrix, converts from X to U; To be computed elsewhere.
    SafeJsonReader reader;
    MecanumDrivebase drivebase;
    LinearOpMode opMode;
    double voltage;
    double lastVoltageSampleTime = -1000; //We don't want to sample voltage that frequently because it changes slowly, and is expensive.

    static class consts{
        static double m; //Robots mass, kg
        static double ke; //Constant of proportionality between current and torque of a motor
        static double kv; //Constant of proportionality between wheel speed (rad/s) and back emf of a motor
        static double r;//Radius of the robot's wheels, (m)
        static double R;//Resistance of the motor + resistance of the battery
        static double I;//rotational intertia of the wheels

    }

    public AutonDriving(MecanumDrivebase drivebase, LinearOpMode opMode){
        reader = new SafeJsonReader("AutoDrivingMat");
        this.drivebase = drivebase;
        this.opMode = opMode;
        readMatrix();
        voltage = getBatteryVoltage();
    }

    //To make the matrix reader look nicer. Letter g chosen pseudorandomly.
    private double g(String name){
        return reader.getDouble(name);
    }

    private void readMatrix(){
        double[][] K_array = {  {g("_0_0"), g("_0_1"), g("_0_2"), g("_0_3"), g("_0_4"), g("_0_5")},
                                {g("_1_0"), g("_1_1"), g("_1_2"), g("_1_3"), g("_1_4"), g("_1_5")},
                                {g("_2_0"), g("_2_1"), g("_2_2"), g("_2_3"), g("_2_4"), g("_2_5")},
                                {g("_3_0"), g("_3_1"), g("_3_2"), g("_3_3"), g("_3_4"), g("_3_5")}
        };
        K = new Matrix(K_array);

    }
    public void setCorrection(Matrix state, Matrix ref){
        double[] torques = K.times(ref.minus(state)).getArrayCopy()[0];
        double[] wheelVels = drivebase.wheelRotVels;
        getBatteryVoltage(); //Call once, so that the value of voltage used is the same for all wheels, even if it is a little off.
        for (int i = 0; i < 4; i++) {
            torques[i] = torqueToVoltage(i, wheelVels[i]); //reuse the same array cause why not
            torques[i] = torques[i] / voltage; //Percentage of available battery voltage to apply to the motor, cause thats how the motors work.
            torques[i] = bound(-1,1, torques[i]); //Power is on [-1, 1]. This call is for safety, ideally it does nothing.
        }

        drivebase.setAllMotorPowers(torques);
    }

    public double  bound(double lower, double upper, double value){
        if (value < lower){
            return lower;
        } else if (value > upper){
            return upper;
        }
        return value;
    }

    public double torqueToVoltage(double torque, double wheel_velocity){
        return torque * consts.R / consts.ke - consts.kv * wheel_velocity;
    }

    double getBatteryVoltage() {
        if (System.currentTimeMillis() - lastVoltageSampleTime < 1000){
            return voltage;
        }
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : opMode.hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        voltage = result;
        lastVoltageSampleTime = System.currentTimeMillis();
        return result;
    }
}
