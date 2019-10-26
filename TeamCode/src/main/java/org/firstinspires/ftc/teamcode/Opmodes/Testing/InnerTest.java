package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Logic.Curves.InnerFollower;

@Autonomous(name = "TestInnerCurves")
public class InnerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        InnerFollower innerFollower = new InnerFollower(0, 0, 30, 30, 45);
        MecanumDrivebase drivebase = new MecanumDrivebase(hardwareMap, telemetry);

        double x = 0, y = 0;
        double lastEncoderPos[];
        double currentEncoderPos[];
        lastEncoderPos = drivebase.getPos();
        currentEncoderPos = drivebase.getPos();
        double xp, yp;
        double temp[];

        waitForStart();
        while (opModeIsActive()){
            if (Math.abs(x - 30) < 1 && Math.abs(y - 30) < 1){
                drivebase.stop();
                drivebase.update();
                telemetry.addLine("Stopped");
                telemetry.update();
                this.requestOpModeStop();
            }
            x -= getDistX(lastEncoderPos, currentEncoderPos);
            y -= getDistY(lastEncoderPos, currentEncoderPos);
            temp = innerFollower.getXY(x, y);
            xp = temp[0];
            yp = temp[0];
            drivebase.drive(xp, yp, 0, false);
            drivebase.update();
            lastEncoderPos = currentEncoderPos;
            currentEncoderPos = drivebase.getPos();
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
