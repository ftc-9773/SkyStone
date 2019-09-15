package org.firstinspires.ftc.teamcode.Opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Logic.CurveTesting;

@TeleOp(name = "CurveThingy")
public class Ritdteleop extends LinearOpMode {
    String TAG = "CURVETESTING";

    @Override
    public void runOpMode() {
        double x, y;
        MecanumDrivebase drivebase = new MecanumDrivebase(hardwareMap, telemetry);
        CurveTesting test = new CurveTesting(hardwareMap, telemetry);

        test.driveinCurve(0,0, 4, 4, 30);

        double x_, y_;
        x_ = 4;
        y_ = 4;

        x = 0;
        y = 0;

        double lastEncoderPos[];
        double currentEncoderPos[];
        double theta = 0;

        double yp, xp;
        lastEncoderPos = drivebase.getPos();
        currentEncoderPos = drivebase.getPos();
        Log.d(TAG, "CX = " + test.h + " CY = " + test.k);
        waitForStart();
        while(opModeIsActive()){
            if (Math.abs(x - x_) < 1 && Math.abs(y - y_) < 1){
                drivebase.stop();
                drivebase.update();
                this.requestOpModeStop();
            }

            theta = Math.acos(1 - Math.sqrt(x * x + y * y) / 2 / test.r / test.r);

            drivebase.drive(Math.sin(theta) / 2, Math.cos(theta) / 2, 0, false);
            drivebase.update();
            x += getDistX(lastEncoderPos, currentEncoderPos);
            y += getDistY(lastEncoderPos, currentEncoderPos);

            lastEncoderPos = currentEncoderPos;
            currentEncoderPos = drivebase.getPos();
            Log.d(TAG, "X = " + x + " Y = " + y + " theta = " + theta * 180 / Math.PI);
        }

    }


    public double getDistX(double[] last, double[] cur){
        double dx = 0;
        dx += ((cur[0] - last[0]) + (cur[1] - last[1])) / 2;
        return dx / MecanumDrivebase.COUNTS_PER_INCH;
    }
    public double getDistY(double[] last, double[] cur){
        double dx = 0;
        dx += ((cur[0] - last[0]) - (cur[1] - last[1])) / 2;
        return dx / MecanumDrivebase.COUNTS_PER_INCH;
    }
}
