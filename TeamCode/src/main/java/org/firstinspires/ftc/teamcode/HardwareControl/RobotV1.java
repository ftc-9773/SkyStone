package org.firstinspires.ftc.teamcode.HardwareControl;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

    Servo leftHook;
    Servo rightHook;

    //TeleOP variables
    double xp, yp, rp;
    double numBlocksHigh = 0;
    Button X = new Button(), B = new Button(), RB = new Button();
    boolean hooksDown = false;
    Telemetry telemetry;

    Intake intake;
    BackHooks backHooks;
    Lifts lifts;

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
        drivebase.drive(xp, yp, rp, true);

        if (gamepad1.left_trigger > 0.05){
            intake.on();
        } else {
            intake.off();
        }

        //Toggle hooks
        RB.recordNewValue(gamepad1.right_bumper);
        if (RB.isJustOff()){


        }

        //Grab or release block
        if (gamepad2.left_bumper){
            lifts.grabBlock();
        } else if (gamepad2.right_bumper){
            lifts.releaseBlock();
        }

        //Rotate claw
        if (gamepad2.dpad_left){
            lifts.rotateClaw90(-1);
        } else if (gamepad2.dpad_right){
            lifts.rotateClaw90(1);
        } else if (Math.abs(gamepad2.right_stick_x) > 0.05){
            lifts.rotateClaw(gamepad2.right_stick_x);
        } else if (gamepad2.left_stick_button){
            lifts.resetClawtoZero();
        }

        //Retract all lifts
        if (gamepad2.a) {
            lifts.resetLifts();
        }
        //Hold Y button to do this.
        if (gamepad2.y) {
            lifts.setHLiftPos(lifts.minPositiveHPos);

        }
        B.recordNewValue(gamepad2.b);
        if (B.isJustOff()){
            numBlocksHigh += 1;
            lifts.setvLiftPos(numBlocksHigh);
        }
        X.recordNewValue(gamepad2.x);
        if (X.isJustOff()) {
            numBlocksHigh -= 1;
            lifts.setvLiftPos(numBlocksHigh);
        }

        if (Math.abs(gamepad2.left_stick_y) > 0.05) {
            lifts.adjustHLift(gamepad2.left_stick_y);
        }
        if (gamepad2.left_trigger > 0.1){
            //lifts.setVLiftPow(gamepad2.left_trigger);
            lifts.adjustVLift(gamepad2.left_trigger);
        } else if (gamepad2.right_trigger > 0.1){
            lifts.adjustVLift(-gamepad2.right_trigger);
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
    }

    public void stop(){}
}
