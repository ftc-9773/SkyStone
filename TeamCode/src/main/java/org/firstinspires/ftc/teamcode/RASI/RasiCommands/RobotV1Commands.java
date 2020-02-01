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

    public void drive(double x, double y){
        double theta = atan(y / x);
        double dist = pow(x * x + y * y, 0.5);
        if (DEBUG) Log.d("ROBOTV1COMMANDS", "Driving dist " + dist  + " at angle " + toDegrees(theta));
        driveUtil.drive(dist, theta);
    }

    public void strafe(double dist){
        if (DEBUG) Log.d("ROBOTV1COMMANDS", "Strafing dist " + dist);
        driveUtil.strafeStraight(dist);
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
