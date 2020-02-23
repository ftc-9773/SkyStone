package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Lifts;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.SideHook;
import org.firstinspires.ftc.teamcode.Utilities.misc.Button;

@TeleOp(name = "Testing Side Hook")
public class SideHookTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Lifts hook = new Lifts(hardwareMap);
        SideHook shook = new SideHook(hardwareMap);
        telemetry.addLine("Waiting for start");
        Button a = new Button();
        Button b = new Button();
        Button x = new Button();
        Button y = new Button();
        Button k = new Button();
        waitForStart();
        while (opModeIsActive()){
            b.recordNewValue(gamepad1.b);
            a.recordNewValue(gamepad1.a);
            x.recordNewValue(gamepad1.x);
            y.recordNewValue(gamepad1.y);
            k.recordNewValue(gamepad1.right_bumper);
//            if (b.isOn()) {
//                hook.resetClawtoZero();
//            } else if (a.isOn()){
//                hook.rotateClaw90();
//            } else if (x.isOn()){
//                hook.rotateClaw180();
//            } else if (Math.abs(gamepad1.right_stick_x) > 0.05) {
//                hook.adjustRotateServo(gamepad1.right_stick_x);
//            }
//            if (y.isOn()){
//                hook.retractHLift();
//            } else if (k.isOn()){
//                hook.extendHLift();
//            }
//            hook.update();
            if (b.isOn()){
                shook.down();
            } else if (a.isOn()){
                shook.up();
            } else if (x.isOn()){
                shook.openHook();
            } else if (y.isOn()){
                shook.closeHook();
            } else if (k.isOn()){
                shook.dropArm();
            } else if (gamepad1.dpad_down){
                shook.hookPlatform();
            }
            shook.update();
        }
    }
}
