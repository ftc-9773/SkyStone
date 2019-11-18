package org.firstinspires.ftc.teamcode.HardwareControl;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.BackHooks;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Intake;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Lifts;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.Utilities.misc.Button;

public class RobotV1 extends Robot {
    double x = 0, y = 0;


    //TeleOP variables
    double xp, yp, rp;
    double numBlocksHigh = 0;
    Button GP2X = new Button(), B = new Button(), RB = new Button(), LB = new Button(), DPDOWN = new Button(), GP1X = new Button(), DPR = new Button(), DPL = new Button(), DPD = new Button();
    boolean hooksDown = false;
    boolean slow = false;
    double drive_direction = 1;
    Telemetry telemetry;

    Intake intake;
    BackHooks backHooks;
    Lifts lifts;
    double clawPos = 0;

    public RobotV1(MecanumDrivebase drivebase, Gyro gyro, Intake intake, Lifts lifts, BackHooks backHooks, Telemetry telemetry){
        this.lifts = lifts;
        this.intake = intake;
        this.gyro = gyro;
        this.drivebase = drivebase;
        this.backHooks = backHooks;
        this.telemetry = telemetry;
    }

    public void runGamepadCommands(Gamepad gamepad1, Gamepad gamepad2){
        xp = gamepad1.left_stick_x;
        yp = gamepad1.left_stick_y;
        rp = gamepad1.right_stick_x;
        if (Math.abs(xp) < 0.06){xp = 0;}
        if (Math.abs(yp) < 0.06){yp = 0;}
        if (Math.abs(rp) < 0.06){rp = 0;}
        GP1X.recordNewValue(gamepad1.x);
        DPDOWN.recordNewValue(gamepad1.dpad_down);
        if (GP1X.isJustOn()){
            drive_direction *= -1;
        }
        if (DPDOWN.isJustOn()){ //Slow mode
            slow = !slow;
        }
        if (slow){ //Slow mode is 60% speed
            xp *= 0.6 * 0.6;
            yp *= 0.6;
            rp *= 0.6;
        }
        drivebase.drive(drive_direction * xp,drive_direction *  -yp, rp, true);

        if (gamepad1.left_trigger > 0.05){
            intake.on();
            lifts.intake();
        } else if (gamepad1.right_trigger > 0.05){
            intake.reverse();
        } else {
            intake.off();
        }
        LB.recordNewValue(gamepad1.left_bumper);
        if (LB.isJustOn()){
            lifts.intake();
        }

        //Toggle hooks
        RB.recordNewValue(gamepad1.right_bumper);
        if (RB.isJustOff()){
            if (hooksDown){
                backHooks.up();
                backHooks.update();
            }else{
                backHooks.down();
                backHooks.update();
            }
            hooksDown = !hooksDown;
        }

        //THE FOLLOWING IS GAMEPAD 2

        //Grab or release block
        if (gamepad2.left_bumper){
            lifts.grabBlock();
        } else if (gamepad2.right_bumper){
            lifts.releaseBlock();
        } else {
            lifts.stopClaw();
        }

        //Rotate claw
        DPL.recordNewValue(gamepad2.dpad_left);
        DPR.recordNewValue(gamepad2.dpad_right);
        DPD.recordNewValue(gamepad2.dpad_down);
        if (DPL.isJustOn()){
            lifts.rotateClaw90();
        } else if (DPR.isJustOn()){
            lifts.rotateClaw180();
        } else if (DPD.isJustOn()){
            lifts.resetClawtoZero();
        }

        //Retract all lifts
        if (gamepad2.a) {
            lifts.resetLifts();
        }
        //Hold Y button to do this.
        if (gamepad2.y) {
            telemetry.addLine("Set to number of blocks " + numBlocksHigh);
            lifts.setvLiftPos(numBlocksHigh);
        }
        B.recordNewValue(gamepad2.b);
        if (B.isJustOff()){
            numBlocksHigh += 1;
            lifts.setvLiftPos(numBlocksHigh);
        }
        GP2X.recordNewValue(gamepad2.x);
        if (GP2X.isJustOff()) {
            numBlocksHigh -= 1;
            if (numBlocksHigh < 0){
                numBlocksHigh = 0;
            }
            lifts.setvLiftPos(numBlocksHigh);
        }

        if (Math.abs(gamepad2.left_stick_y) > 0.05) {
            //lifts.adjustHLift(gamepad2.left_stick_y);
            lifts.setHLiftPow(0.3 * gamepad2.left_stick_y);
        } else {
            lifts.setHLiftPow(0);
        }
        if (Math.abs(gamepad2.right_stick_y) > 0.05){
            lifts.adjustVLift(-gamepad2.right_stick_y);
            //lifts.setvLiftPow(gamepad2.right_stick_y);
        } else {
            //lifts.setvLiftPow(0);
        }

    }

    public void setIntake(boolean on){
        if (on) intake.on();
        else intake.off();
    }


    @Override
    public void update() {
        drivebase.update();
        intake.update();
        heading = getHeading();
        lifts.update();
        backHooks.update();
    }

    public void stop(){}
}
