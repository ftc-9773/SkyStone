package org.firstinspires.ftc.teamcode.Opmodes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Lifts;

@TeleOp(name="TestRotateServo")
public class test extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Lifts lifts = new Lifts(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a){
                lifts.resetClawtoZero();
            } else if(gamepad1.b) {
                lifts.rotateClaw90();
            } else if (gamepad1.x){
                lifts.rotateClaw180();
            }
            lifts.update();
        }
    }
}
