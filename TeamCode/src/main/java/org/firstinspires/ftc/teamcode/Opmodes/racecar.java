package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "RaceCar")
public class racecar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor m0, m2, m3, m1;
        m0 = hardwareMap.get(DcMotor.class, "0");
        m1 = hardwareMap.get(DcMotor.class, "1");
        m2 = hardwareMap.get(DcMotor.class, "2");
        m3 = hardwareMap.get(DcMotor.class, "3");

        waitForStart();

        while(opModeIsActive()){
            m0.setPower(gamepad1.left_stick_y > 0.1? gamepad1.left_stick_y: 0);
            m1.setPower(gamepad1.left_stick_y > 0.1? gamepad1.left_stick_y: 0);
            m2.setPower(gamepad1.right_stick_y > 0.1? gamepad1.right_stick_y: 0);
            m3.setPower(gamepad1.right_stick_y > 0.1? gamepad1.right_stick_y: 0);
        }
    }


}
