package org.firstinspires.ftc.teamcode.HardwareControl;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.BackHooks;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Intake;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Lifts;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.SideHook;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.Utilities.misc.Button;

public class RobotV1 extends Robot {
    double x = 0, y = 0;


    //TeleOP variables
    double xp, yp, rp;
    public double yLiftHeight = 0.0, bLiftHeight = 0;
    boolean bLastClicked = false, yLastClicked = false;
    Button GP1A = new Button(), GP1X = new Button(), GP1B = new Button(), A = new Button(), B = new Button(), Y = new Button(),  RB = new Button(), LB = new Button(), GP1_DPLEFT = new Button(), GP1_DPRIGHT = new Button(), GP2X = new Button(), DPR = new Button(), GP2_LStick = new Button(), DPU = new Button(), GP2_DPDOWN = new Button(), RB2 = new Button();
    Button GP1DPD = new Button(), GP1DPU = new Button();
    boolean hooksDown = false;
    boolean capstoneClick = true;
    boolean hLiftRetracted = true;
    boolean slow = false;
    boolean intakeLoaded, invert = false, regular = true;
    boolean releaseCapstone = false;
    double drive_direction = 1;
    Telemetry telemetry;

    public Intake intake;
    BackHooks backHooks;
    Lifts lifts;
    SideHook sideHook;
    double clawPos = 0;



    public RobotV1(MecanumDrivebase drivebase, Gyro gyro, Intake intake, Lifts lifts, BackHooks backHooks, Telemetry telemetry, SideHook sideHook){
        this.lifts = lifts;
        this.intake = intake;
        this.gyro = gyro;
        this.drivebase = drivebase;
        this.backHooks = backHooks;
        this.telemetry = telemetry;
        this.sideHook = sideHook;
    }


    public void runGamepadCommands(Gamepad gamepad1, Gamepad gamepad2) {
        //THE FOLLOWING IS GAMEPAD 1


        xp = gamepad1.left_stick_x * 0.9;
        yp = gamepad1.left_stick_y * 0.9;
        rp = gamepad1.right_stick_x * 0.9;

        if (Math.abs(xp) < 0.0001) {
            xp = 0;
        }
        if (Math.abs(yp) < 0.0001) {
            yp = 0;
        }
        if (Math.abs(rp) < 0.01) {
            rp = 0;
        }

        GP1X.recordNewValue(gamepad1.x);
        GP1B.recordNewValue(gamepad1.b);
        GP1_DPLEFT.recordNewValue(gamepad1.left_bumper);
        GP1_DPRIGHT.recordNewValue(gamepad1.dpad_right);

        if (intake.isLoaded()) {
            invert = true;
            regular = false;
        }
        if (GP1X.isJustOn()) {
              invert = true;
              regular = false;
        }
        if (GP1B.isJustOn()) {
            regular = true;
            invert = false;
        }
        if (invert) {
            drive_direction = -1;
        }
        if (regular) {
            drive_direction = 1;
        }

        if (gamepad1.right_bumper) {
            slow = true;
        }
        else {
            slow = false;

        }

        if (slow) { //Slow mode is 60% speed
            rp *= 0.65;
            if (Math.abs(xp) < 0.001) {
                xp *= 600;
            }
            else {
                xp *= 0.575;
            }
            if (Math.abs(yp) < 0.001) {
                yp *= 600;
            }
            else {
                yp *= 0.575;
            }
        }
        drivebase.drive(drive_direction * xp, drive_direction * -yp, rp, true);

        GP1A.recordNewValue(gamepad1.a);
        if (GP1A.isJustOff()) {
            sideHook.up();
        }
        else {
            sideHook.down();
        }


        if (gamepad1.left_trigger > 0.05) {
            lifts.intake();
            intake.on();
        } else if (gamepad1.right_trigger > 0.05) {
            lifts.intake();
            intake.onReverse();
        } else {
            intake.off();
        }

        //Toggle hooks
        if (gamepad1.left_bumper) {
            backHooks.down();
            backHooks.update();
        } else {
            backHooks.up();
            backHooks.update();
        }
        hooksDown = !hooksDown;



        GP1DPU.recordNewValue(gamepad1.dpad_up);
        GP1DPD.recordNewValue(gamepad1.dpad_down);

//        if (GP1DPD.isOn()){
//            lifts.disablePIDLiftControl();
//            lifts.setvLiftPow(-0.1);
//        } else if (GP1DPU.isOn()){
//            lifts.disablePIDLiftControl();
//            lifts.setvLiftPow(0.1);
//        } else {
//            lifts.enablePIDLiftControl();
//        }

        //THE FOLLOWING IS GAMEPAD 2

        //Grab or release block
        if (gamepad2.right_trigger > 0.05) {
            lifts.releaseBlock();
        } else if (gamepad2.left_trigger > 0.05) {
            lifts.grabBlock();
        } else {
            lifts.stopClaw();
        }

        //Rotate claw
        DPU.recordNewValue(gamepad2.dpad_up);
        DPR.recordNewValue(gamepad2.dpad_right);
        GP2_DPDOWN.recordNewValue(gamepad2.dpad_down);
        if (DPR.isJustOn()) {
            lifts.rotateClaw90();
        } else if (DPU.isJustOn()) {
            lifts.rotateClaw180();
        } else if (GP2_DPDOWN.isJustOn()) {
            lifts.resetClawtoZero();
            //lifts.setHLiftPos(lifts.gethLiftMaxPos());
        }

        GP2_LStick.recordNewValue(gamepad2.left_stick_button);
        if (GP2_LStick.isJustOn()) {
            if (capstoneClick) {
                lifts.releaseCapstone();
                capstoneClick = false;
            } else {
                lifts.resetCapstone();
                capstoneClick = true;
            }
        }

        //Retract all lifts
        A.recordNewValue(gamepad2.a);
        if (A.isJustOff()) {

            long startTime = System.currentTimeMillis();
            while (startTime + 400 > System.currentTimeMillis()){
                lifts.releaseBlock();
                update();
            }

            lifts.setvLiftPos(lifts.getVliftPos() + 400);
            startTime = System.currentTimeMillis();
            while (startTime + 375 > System.currentTimeMillis()){
                update();
            }

            startTime = System.currentTimeMillis();
            while (startTime + 700 > System.currentTimeMillis()){
                lifts.retractHLift();
                update();
            }

            lifts.setvLiftPos(5);
            update();
            hLiftRetracted = true;
            bLiftHeight = 0.0;
        }

        B.recordNewValue(gamepad2.b);
        if (B.isJustOff()) {
            bLastClicked = true;
            yLastClicked = false;
            bLiftHeight += 1.0;
            if (bLiftHeight <= 10.0) {
                lifts.setvLiftPos(bLiftHeight);
            } else {
                bLiftHeight = 10.0;
                lifts.setvLiftPos(bLiftHeight);
            }
        }

        GP2X.recordNewValue(gamepad2.x);
        if (GP2X.isJustOff()) {
            if (bLastClicked) {
                bLiftHeight -= 1.0;
                if (bLiftHeight >= 0.0) {
                    lifts.setvLiftPos(bLiftHeight);
                } else {
                    bLiftHeight = 0.0;
                    lifts.setvLiftPos(bLiftHeight);
                }
            }
            if (yLastClicked) {
                yLiftHeight -= 1.0;
                if (yLiftHeight >= 0.0) {
                    lifts.setvLiftPos(yLiftHeight);
                } else {
                    yLiftHeight = 0.0;
                    lifts.setvLiftPos(yLiftHeight);
                }
            }
        }

        Y.recordNewValue(gamepad2.y);
        if (Y.isJustOff()) {
            bLastClicked = false;
            yLastClicked = true;
            yLiftHeight += 1.0;
            if (yLiftHeight <= 10.0) {
                lifts.setvLiftPos(yLiftHeight);
                lifts.grabBlock();
            } else {
                yLiftHeight = 10.0;
                lifts.setvLiftPos(yLiftHeight);
            }
            backHooks.up();
            backHooks.update();
        }


//        if (Math.abs(gamepad2.left_stick_y) > 0.05) {
//            //lifts.adjustHLift(gamepad2.left_stick_y);
//            lifts.setHLiftPow(0.5 * gamepad2.left_stick_y);
//        } else {
//            lifts.setHLiftPow(0);
//        }

//        if (gamepad2.left_stick_y > 0.05) {
//            lifts.extendHLift();
//        }
//
//        if (gamepad2.left_stick_y < -0.05) {
//            lifts.retractHLift();
//        }

        if (Math.abs(gamepad2.right_stick_y) > 0.05) {
            lifts.adjustVLift(-gamepad2.right_stick_y);
            //lifts.setvLiftPow(gamepad2.right_stick_y);
        } else {
            //lifts.setvLiftPow(0);
        }

        if (hLiftRetracted) {
            lifts.retractHLift();
        }
        else {
            lifts.extendHLift();
        }

        RB2.recordNewValue(gamepad2.right_bumper);
        if (RB2.isJustOff()) {
            if (hLiftRetracted) {
                lifts.extendHLift();
            }
            else {
                lifts.retractHLift();
            }
            hLiftRetracted = !hLiftRetracted;
        }

        if (gamepad2.left_bumper) {
            lifts.grabBlock();
            lifts.setvLiftPos(0);
        }
    }

    public void driveFast(double speed) {
            drivebase.drive(0, speed, 0, false);
    }

    public void stopDriving() {
        drivebase.stop();
    }

    public void stop(){}

    boolean intakeOn;

    public void setIntake(boolean on){
        intakeOn = on;
        if (intakeOn) intake.on();
        else intake.off();
    }

    public void setReverseIntake(boolean on){
        intakeOn = on;
        if (intakeOn) intake.onReverse();
        else intake.off();
    }

    public void dropHooks(){
        backHooks.down();
        hooksDown = true;
    }

    public void upHooks(){
        backHooks.up();
        hooksDown = false;
    }

    public double getGearRatio() {
        return lifts.getGearRatio();
    }

    public void grab(){
        lifts.grabBlock();
    }

    public void release(){
        lifts.releaseBlock();
    }

    public void clawOff(){
        lifts.stopClaw();
    }

    public int getvLiftIntakePos() { return lifts.getIntakePos(); }

    public void setVLiftPos(int pos){
        lifts.setvLiftPos(pos);
    }

    public void setVLiftPos(double pos) {lifts.setvLiftPos(pos);}

    public int getBlockHeightInEncoders(){ return lifts.getBlockHeightInEncoders();}

    //public void moveHLift(double pow) {lifts.setHLiftPow(pow);}

    public void extendHLift() {
        lifts.extendHLift();
        lifts.update();
    }

    public void retractHLift() {
        lifts.retractHLift();;
        lifts.update();
    }

    public void sideHookUp(){
        sideHook.up();
        sideHook.update();
    }

    public void sideHookDown(){
        sideHook.down();
        sideHook.update();
    }

    public void update() {
        drivebase.update();
        intake.update();
        heading = getHeading();
        lifts.update();
        backHooks.update();
        sideHook.update();
    }
}

