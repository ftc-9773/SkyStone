package org.firstinspires.ftc.teamcode.RASI.RasiCommands;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.RobotV1;
import org.firstinspires.ftc.teamcode.Logic.DriveUtil;
import org.firstinspires.ftc.teamcode.Utilities.misc.Timer;

import static java.lang.Math.*;


public class RobotV1Commands extends RasiCommands{
    private static final boolean DEBUG = false;
    RobotV1 robot;
    DriveUtil driveUtil;

    public RobotV1Commands(LinearOpMode opMode, RobotV1 robot){
        super(opMode);
        this.robot = robot;
        this.driveUtil = new DriveUtil(robot, opMode);
    }

    public void driveFast(double time, double speed, double afterDistance) {
        long initialTime = System.currentTimeMillis();
        while(initialTime + time*1000 > System.currentTimeMillis()) {
            robot.update();
            robot.driveFast(speed);
            robot.update();
        }
        robot.driveFast(0);
        driveUtil.driveStraight(afterDistance, 1);
        robot.update();
    }

    public void piddrive(double dist){
        driveUtil.PIDdriveForward(dist, 1);
    }

    public void drivePower(double power, double time){
        robot.drivebase.drive(0, power, 0, false);
        wait(time);
        robot.drivebase.stop();
        robot.update();
    }

    public void drive(double y){
        driveUtil.PIDdriveForward(y, 1);
    }

    public void strafe(double x){
        driveUtil.PIDdriveStrafe(x, 1);
    }

    public void dropClaw(){
        robot.sideHook.down();
        robot.sideHook.openHook();
    }

    public void grabClaw(){
        robot.sideHook.closeHook();
    }
    public void clawUp(){
        robot.sideHook.up();
    }
    public void fullretractclaw(){
        robot.sideHook.fullRetract();
    }

    //Time based driving
    public void driveFast2(double time, double speed, double afterDistance) {
        long initialTime = System.currentTimeMillis();
        while(initialTime + time*1000 > System.currentTimeMillis()) {
            robot.update();
            robot.driveFast(speed);
            robot.update();
        }
        robot.driveFast(0);
        driveUtil.driveStraight(afterDistance, 0.5);
        robot.update();
    }

    public void driveHoldHeading(){
        double heading = robot.getHeading();
        Timer a = new Timer(10);
        while(!a.isDone()){
            driveUtil.driveHoldHeading(0, 0, heading);
            robot.update();
        }
    }

    public void driveStraight(double dist){
        if (DEBUG) Log.d("ROBOTV1COMMANDS", "DrivingStraight dist" + dist);
        driveUtil.driveStraight(dist, 1);
    }
    public void driveSlow(double dist, double coe){
        if (DEBUG) Log.d("ROBOTV1COMMANDS", "DrivingStraight dist" + dist);
        driveUtil.driveStraight(dist, 0.5);
    }

    public void dropHooks(){
        robot.dropHooks();
    }

    public void upHooks(){
        robot.upHooks();
    }

    public void grabPlatform(){
        robot.sideHook.hookPlatform();
    }

    public void driveIntoWall(double power ){
            robot.drivebase.drive(power, 0, 0, false);
            double init_x, init_y;
            init_x = robot.x;
            init_y = robot.y;
            double dt = 0.00001;
            long lastReadTime = System.currentTimeMillis();
            robot.update();
            boolean firstTime = true;
            while (driveUtil.dist(init_x, init_y, robot.x, robot.y) / dt > 0.05 || firstTime){
                firstTime = false;
                dt = System.currentTimeMillis() - lastReadTime;
                lastReadTime = System.currentTimeMillis();
                robot.drivebase.drive(power, 0, 0, false);
                robot.update();

            }
    }

    public void strafePower(double power, double time){
        robot.drivebase.drive(power, 0, 0, false);
        wait(time);
    }

    public void turn(double angle){
        driveUtil.turnToAngle(angle);
    }

    public void wait(double time){
        long startTime = System.currentTimeMillis();
        while(startTime + time*1000 > System.currentTimeMillis() && !opMode.isStopRequested()){
            robot.update();
        }
    }

    public void absIntakeOn(){
        robot.setIntake(true);
    }

    public void intakeOn(){
        robot.setIntake(true);
        robot.intake.autoOff = true;
    }

    public void reverseIntakeOn() {
        robot.setReverseIntake(true);
        robot.update();

    }

    //Intake and then grab a block, then outtake to make sure there aren't two blocks.
    public void grabBlock(double dist){
        vLiftIntakePos();
        wait(0.05);
        intakeOn();
        driveStraight(dist);
        retractHLift();
        wait(0.01);
        intakeOff();
    }

    public void intakeOff(){
        robot.setIntake(false);
        robot.setReverseIntake(false);
        robot.intake.autoOff = false;
    }

    public void sideHookDown() {
        robot.sideHookDown();
        robot.update();
    }

    public void sideHookUp() {
        robot.sideHookUp();
        robot.update();
    }

    public void vLiftDown(){
        robot.setVLiftPos(0);
    }

    public void vLiftIntakePos() { robot.setVLiftPos(robot.getvLiftIntakePos());}

    public void vLiftRaise(double blocks) { robot.setVLiftPos(blocks); }

//    public void moveHLift(double time, double power) {
//        power = -power;
//        long startTime2 = System.currentTimeMillis();
//        while (startTime2 + time * 1000 > System.currentTimeMillis() && !opMode.isStopRequested()) {
//            robot.update();
//            robot.moveHLift(power);
//            robot.update();
//        }
//    }

    public void extendHLift() {
        robot.extendHLift();
        robot.update();
    }

    public void retractHLift() {
        robot.retractHLift();
        robot.update();
    }

    public void highpowerturn(double ang, double pow){
        driveUtil.turnToAngleHighPower(ang, pow);
    }

    public void grabWNoTime(){
        long startTime3 = System.currentTimeMillis();
        while (startTime3 + 0.5 * 1000 > System.currentTimeMillis() && !opMode.isStopRequested()) {
            robot.grab();
            robot.update();
        }
        robot.clawOff();
        robot.update();
    }

    public void grabWTime() {
        retractHLift();
        vLiftDown();
        retractHLift();
        robot.grab();
        wait(0.5);
        robot.clawOff();
    }

    public void turnClawOn(){
        robot.grab();
    }

    public void turnClawOff(){
        robot.clawOff();
    }

    public void release(){
        long startTime4 = System.currentTimeMillis();
        while (startTime4 + 0.5 * 1000 > System.currentTimeMillis() && !opMode.isStopRequested()) {
            robot.release();
            robot.update();
        }
        robot.clawOff();
        robot.update();
    }

    public void curve(double x, double y, double theta){
        driveUtil.TankCurve(x, y, theta);
    }

}
