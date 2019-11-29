package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class OdemetryWheelTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor e0;
        DcMotor e1;
        DcMotor e2;

        e0 = hardwareMap.get(DcMotor.class, "e0");
        e1 = hardwareMap.get(DcMotor.class, "e1");
        e2 = hardwareMap.get(DcMotor.class, "e2");

        waitForStart();

        while(opModeIsActive()){
            telemetry.addLine("e0 " + e0.getCurrentPosition());
            telemetry.addLine("e1" + e1.getCurrentPosition());
            telemetry.addLine("e2" + e2.getCurrentPosition());
            telemetry.update();
        }
    }
}
