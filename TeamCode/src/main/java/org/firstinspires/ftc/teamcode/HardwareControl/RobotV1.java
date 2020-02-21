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
import com.qualcomm.robotcore.hardware.DigitalChannel;

import android.util.Log;

public class RobotV1 extends Robot {
    double x = 0, y = 0;


    //TeleOP variables
    double xp, yp, rp;
    public double yLiftHeight = 0.0, bLiftHeight = 0;
    boolean bLastClicked = false, yLastClicked = false;
    Button GP1A = new Button(), GP1X = new Button(), GP1B = new Button(), GP2A = new Button(), GP2B = new Button(), GP2Y = new Button(),  RB = new Button(), LB = new Button(), GP1_DPLEFT = new Button(), GP1_DPRIGHT = new Button(), GP2X = new Button(), GP2DPR = new Button(), GP2_LStick = new Button(), GP2DPU = new Button(), GP2DPD = new Button(), GP2RB = new Button();
    Button GP1DPD = new Button(), GP1DPU = new Button();
    boolean hooksDown = false;
    boolean capstoneClick = true;
    boolean hLiftRetracted = true;
    boolean slow = false;
    boolean intakeLoaded, invert = false, regular = true;
    boolean releaseCapstone = false;
    boolean outtakeClosePos = false;
    double drive_direction = 1;
    Telemetry telemetry;

    public Intake intake;
    BackHooks backHooks;
    Lifts lifts;
    SideHook sideHook;
    double clawPos = 0;

    static boolean DEBUG = true;

    public RobotV1(MecanumDrivebase drivebase, Gyro gyro, Intake intake, Lifts lifts, BackHooks backHooks, Telemetry telemetry, SideHook sideHook){
        super(drivebase, gyro);
        this.lifts = lifts;
        this.intake = intake;
        this.gyro = gyro;
        this.drivebase = drivebase;
        this.backHooks = backHooks;
        this.telemetry = telemetry;
        this.sideHook = sideHook;
        sideHook.up();
        lifts.resetClawtoZero();
    }


    public void runGamepadCommands(Gamepad gamepad1, Gamepad gamepad2) {
        //THE FOLLOWING IS GAMEPAD 1
        double lastTime = System.currentTimeMillis();
        double timetracking = lastTime;
        xp = gamepad1.left_stick_x * 0.9;
        yp = gamepad1.left_stick_y * 0.9;
        rp = gamepad1.right_stick_x * 0.9;

        GP1X.recordNewValue(gamepad1.x);
        GP1B.recordNewValue(gamepad1.b);
        GP1_DPLEFT.recordNewValue(gamepad1.left_bumper);
        GP1_DPRIGHT.recordNewValue(gamepad1.dpad_right);
        GP1A.recordNewValue(gamepad1.a);
        GP1DPU.recordNewValue(gamepad1.dpad_up);
        GP1DPD.recordNewValue(gamepad1.dpad_down);
        GP2DPU.recordNewValue(gamepad2.dpad_up);
        GP2DPR.recordNewValue(gamepad2.dpad_right);
        GP2DPD.recordNewValue(gamepad2.dpad_down);
        GP2_LStick.recordNewValue(gamepad2.left_stick_button);
        GP2A.recordNewValue(gamepad2.a);
        GP2B.recordNewValue(gamepad2.b);
        GP2X.recordNewValue(gamepad2.x);
        GP2Y.recordNewValue(gamepad2.y);
        GP2RB.recordNewValue(gamepad2.right_bumper);
        timetracking = System.currentTimeMillis();
        Log.d("RobotV1", "GamepadReading " + (timetracking - lastTime));


        if (Math.abs(xp) < 0.0001) {
            xp = 0;
        }
        if (Math.abs(yp) < 0.0001) {
            yp = 0;
        }
        if (Math.abs(rp) < 0.01) {
            rp = 0;
        }

        long magic = System.currentTimeMillis();
        if (intake.isLoaded()) {
            invert = true;
            regular = false;
        }
        Log.d("RobotV1", "IsLoaded " + (System.currentTimeMillis()- magic));

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
        Log.d("RobotV1", "DrivingCode " + (System.currentTimeMillis() - timetracking));
        timetracking = System.currentTimeMillis();


        drivebase.drive(drive_direction * xp, drive_direction * -yp, rp, true);
        Log.d("RobotV1", "DrivebaseWriting " + (System.currentTimeMillis() - timetracking));
        timetracking = System.currentTimeMillis();

        //Make a control for the side claw


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

//        if (GP1DPD.isOn()){
//            lifts.disablePIDLiftControl();
//            lifts.setvLiftPow(-0.1);
//        } else if (GP1DPU.isOn()){
//            lifts.disablePIDLiftControl();
//            lifts.setvLiftPow(0.1);
//        } else {
//            lifts.enablePIDLiftControl();
//        }
        Log.d("RobotV1", "Intake + Hooks " + (System.currentTimeMillis() - timetracking));
        timetracking = System.currentTimeMillis();

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

        if (GP2DPD.isJustOn()) {
            lifts.resetClawtoZero();
        } else if (Math.abs(gamepad2.right_stick_x) > 0.1){
            lifts.adjustRotateServo(gamepad2.right_stick_x);
        }
        if (GP2_LStick.isJustOn()) {
            if (capstoneClick) {
                lifts.releaseCapstone();
                capstoneClick = false;
            } else {
                lifts.resetCapstone();
                capstoneClick = true;
            }
        }
        Log.d("RobotV1", "ClawCode " + (System.currentTimeMillis() - timetracking));
        timetracking = System.currentTimeMillis();


        //Retract all lifts

        if (GP2A.isJustOff()) {
            lifts.resetClawtoZero();
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

            AStartTime = System.currentTimeMillis();
            while (AStartTime + 700 > System.currentTimeMillis()){
                lifts.retractHLift();
                update();
            }

            lifts.setvLiftPos(5);
            update();
            hLiftRetracted = true;
            bLiftHeight = 0.0;
        }

        if (GP2B.isJustOff()) {
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

        if (GP2Y.isJustOff()) {
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

        if (Math.abs(gamepad2.right_stick_y) > 0.07) {
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

        if (GP2RB.isJustOff()) {
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
        Log.d("RobotV1", "GamepadLoopTime " + (System.currentTimeMillis() - lastTime));
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
        double lastTime = System.currentTimeMillis();
        double thisTime;
        drivebase.update();
        thisTime = System.currentTimeMillis();
        //if (DEBUG) Log.d("RobotV1", "drivebase update time millis: " + (thisTime - lastTime));
        lastTime = thisTime;
        intake.update();
        thisTime = System.currentTimeMillis();
        //if (DEBUG) Log.d("RobotV1", "intake update time millis: " + (thisTime - lastTime));
        lastTime = thisTime;
        heading = getHeading();
        updateposition();
        thisTime = System.currentTimeMillis();
        //if (DEBUG) Log.d("RobotV1", "getheading() update time millis: " + (thisTime - lastTime));
        lastTime = thisTime;
        lifts.update();
        thisTime = System.currentTimeMillis();
        //if (DEBUG) Log.d("RobotV1", "lifts update time millis: " + (thisTime - lastTime));
        lastTime = thisTime;
        backHooks.update();
        thisTime = System.currentTimeMillis();
        //if (DEBUG) Log.d("RobotV1", "backhooks update time millis: " + (thisTime - lastTime));
        lastTime = thisTime;
        sideHook.update();
        thisTime = System.currentTimeMillis();
        //if (DEBUG) Log.d("RobotV1", "sideHook update time millis: " + (thisTime - lastTime));

    }
}

