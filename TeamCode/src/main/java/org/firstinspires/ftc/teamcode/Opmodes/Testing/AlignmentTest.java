package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Lifts;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.HardwareControl.Robot;
import org.firstinspires.ftc.teamcode.Logic.DriveUtil;

@TeleOp(name = "Measurements")
public class AlignmentTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Lifts lifts = new Lifts(hardwareMap);
        sendTelemetry("everything created");
        double ot = System.currentTimeMillis();
        double dt = 0;
        double os = lifts.getVliftPos();
        double ds = 0;

        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.left_stick_y > 0.05){
                //lifts.setVLiftPow(gamepad1.left_stick_y);
            }
            if (!gamepad1.x){
                dt = ot - System.currentTimeMillis();
                ds = lifts.getVliftPos() - os;
            }
            telemetry.addLine("dt " + dt);
            telemetry.addLine("ds " + ds);
            telemetry.addLine("v " + ds / dt);
            telemetry.update();
            if (gamepad1.y){
                ot = System.currentTimeMillis();
                os = lifts.getVliftPos();
            }
            lifts.update();
        }
    }

    public void sendTelemetry(String msg){
        telemetry.addLine(msg);
        telemetry.update();
    }
}
