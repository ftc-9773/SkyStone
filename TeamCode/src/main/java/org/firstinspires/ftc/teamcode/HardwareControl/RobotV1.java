package org.firstinspires.ftc.teamcode.HardwareControl;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.BackHooks;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Intake;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Lifts;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.DistSensor;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.Utilities.misc.Button;

public class RobotV1 extends Robot {
    double x = 0, y = 0;


    //TeleOP variables
    double xp, yp, rp;
    double numBlocksHigh = 0;
    Button GP1X = new Button(), GP1B = new Button(), A = new Button(), B = new Button(), Y = new Button(),  RB = new Button(), LB = new Button(), GP1_DPLEFT = new Button(), GP1_DPRIGHT = new Button(), GP2X = new Button(), DPR = new Button(), DPU = new Button(), GP2_DPDOWN = new Button();
    boolean hooksDown = false;
    double currentLiftHeight = 0;
    boolean slow = false;
    boolean releaseCapstone = false;
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

    public RobotV1(MecanumDrivebase drivebase, Gyro gyro, Intake intake, Lifts lifts, DistSensor distSensor, BackHooks backHooks, Telemetry telemetry){
        this.lifts = lifts;
        this.intake = intake;
        this.gyro = gyro;
        this.drivebase = drivebase;
        this.backHooks = backHooks;
        this.telemetry = telemetry;
    }
    public void runGamepadCommands(Gamepad gamepad1, Gamepad gamepad2){
              //THE FOLLOWING IS GAMEPAD 1

        xp = gamepad1.left_stick_x;
        yp = gamepad1.left_stick_y;
        rp = gamepad1.right_stick_x;

        if (Math.abs(xp) < 0.0001){xp = 0;}
        if (Math.abs(yp) < 0.0001){yp = 0;}
        if (Math.abs(rp) < 0.01){rp = 0;}

        GP1X.recordNewValue(gamepad1.x);
        GP1B.recordNewValue(gamepad1.b);
        GP1_DPLEFT.recordNewValue(gamepad1.left_bumper);
        GP1_DPRIGHT.recordNewValue(gamepad1.dpad_right);

        if (GP1X.isJustOn()){
            drive_direction = -1;
        }
        if (GP1B.isJustOn()){
            drive_direction = 1;
        }

        if (GP1_DPLEFT.isJustOn()){ //Slow mode
            slow = true;
        }
        if (GP1_DPRIGHT.isJustOn()){
            slow = false;
        }
        if (slow){ //Slow mode is 60% speed
            xp *= 0.4;
            yp *= 0.4;
            rp *= 0.65;
        }
        drivebase.drive(drive_direction * xp,drive_direction *  -yp, rp, true);


        if (gamepad1.left_trigger > 0.05){
            intake.on();
            lifts.intake();
        } else if (gamepad1.right_trigger > 0.05){
            intake.onReverse();
        } else {
            intake.off();
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


        //Sets List Intake Position
        if (gamepad2.left_trigger > 0.05){
            lifts.intake();
            currentLiftHeight = 0.0;
        }

        //Grab or release block
        if (gamepad2.left_bumper){
            lifts.grabBlock();
        } else if (gamepad2.right_bumper){
            lifts.releaseBlock();
        } else {
            lifts.stopClaw();
        }

        //Rotate claw
        DPU.recordNewValue(gamepad2.dpad_up);
        DPR.recordNewValue(gamepad2.dpad_right);
        GP2_DPDOWN.recordNewValue(gamepad2.dpad_down);
        if (DPR.isJustOn()){
            lifts.rotateClaw90();
        } else if (DPU.isJustOn()){
            lifts.rotateClaw180();
        } else if (GP2_DPDOWN.isJustOn()){
            lifts.resetClawtoZero();
            //lifts.setHLiftPos(lifts.gethLiftMaxPos());
        }

        //Retract all lifts
        A.recordNewValue(gamepad2.a);
        if (A.isJustOff()) {
            lifts.resetLifts();
            currentLiftHeight = 0.0;
        }
        //Left Stick button to do this.
        if (gamepad2.left_stick_button) {
            telemetry.addLine("Set to number of blocks " + numBlocksHigh);
        }

        B.recordNewValue(gamepad2.b);
        if (B.isJustOff()){
            currentLiftHeight += 1.0;
            if (((int)currentLiftHeight)* lifts.getBlockHeightInEncoders() + 670 < lifts.getvLiftMaxPos()){
                lifts.setvLiftPos(currentLiftHeight);
            }
            else {
                currentLiftHeight = (double)((lifts.getvLiftMaxPos() - 670) / lifts.getBlockHeightInEncoders());
            }

        }

        GP2X.recordNewValue(gamepad2.x);
        if (GP2X.isJustOff()) {
            currentLiftHeight -= 1.0;
            if (currentLiftHeight >= 0.0){
                lifts.setvLiftPos(currentLiftHeight);
            }
            else {
                currentLiftHeight = 0.0;
            }
        }

        Y.recordNewValue(gamepad2.y);
        if (Y.isJustOff()){
            numBlocksHigh += 1;
            lifts.setvLiftPos(numBlocksHigh);
        }


        if (Math.abs(gamepad2.left_stick_y) > 0.05) {
            //lifts.adjustHLift(gamepad2.left_stick_y);
            lifts.setHLiftPow(0.5 * gamepad2.left_stick_y);
        } else {
            lifts.setHLiftPow(0);
        }

        if (Math.abs(gamepad2.right_stick_y) > 0.05){
            lifts.adjustVLift(-gamepad2.right_stick_y);
            //lifts.setvLiftPow(gamepad2.right_stick_y);
        } else {
            //lifts.setvLiftPow(0);
        }


        if (gamepad2.right_trigger > 0.05) {
            lifts.releaseCapstone();
        } else {
            lifts.resetCapstone();
        }

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

    public void grab(){
        lifts.grabBlock();
    }

    public void release(){
        lifts.releaseBlock();
    }

    public void clawOff(){
        lifts.stopClaw();
    }

    public void setVLiftPos(int pos){
        lifts.setvLiftPos(pos);
    }

    public int getBlockHeightInEncoders(){ return lifts.getBlockHeightInEncoders();}

    public void extendHLift() { lifts.setHLiftPos(lifts.gethLiftMaxPos());}

    public void retractHLift() {lifts.setHLiftPos(lifts.gethLiftZeroPos());}
}

