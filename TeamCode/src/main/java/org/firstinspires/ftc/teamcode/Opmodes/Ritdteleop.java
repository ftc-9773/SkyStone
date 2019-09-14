package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Logic.CurveTesting;

@TeleOp(name = "WhyHelloThere")
public class Ritdteleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        double x, y;
        MecanumDrivebase drivebase = new MecanumDrivebase(hardwareMap, telemetry);
        CurveTesting test = new CurveTesting(hardwareMap, telemetry);

        test.driveinCurve(0,0, 10, 10, 45);

        double x_, y_;
        x_ = 10;
        y_ = 10;

        x = 0;
        y = 0;

        double lastEncoderPos[];
        double currentEncoderPos[];

        double yp, xp;
        lastEncoderPos = drivebase.getPos();
        currentEncoderPos = drivebase.getPos();
        waitForStart();
        while(opModeIsActive()){
            if (Math.abs(x - x_) < 1 && Math.abs(y - y_) < 1){
                drivebase.stop();
                drivebase.update();
                this.requestOpModeStop();
            }

            yp = test.y_prime(x, y);
            xp = test.x_prime(x, y);
            if (Math.max(xp, yp) > 1){
                double max = Math.max(xp, yp);
                yp = yp / Math.max(xp, yp);
                xp = xp /max;
            }
            drivebase.drive(xp, yp, 0, false);
            drivebase.update();
            x += getDistX(lastEncoderPos, currentEncoderPos);
            y += getDistY(lastEncoderPos, currentEncoderPos);

            lastEncoderPos = currentEncoderPos;
            currentEncoderPos = drivebase.getPos();
        }

    }


    public double getDistX(double[] last, double[] cur){
        double dx = 0;
        dx += ((cur[0] - last[0]) + (cur[1] - last[1])) / 2;
        return dx;
    }
    public double getDistY(double[] last, double[] cur){
        double dx = 0;
        dx += ((cur[0] - last[0]) - (cur[1] - last[1])) / 2;
        return dx;
    }
}
