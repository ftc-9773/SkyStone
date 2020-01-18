package org.firstinspires.ftc.teamcode.RASI.RasiCommands;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.RobotV1;
import org.firstinspires.ftc.teamcode.Logic.DriveUtil;
import static java.lang.Math.*;


public class RobotV1Commands extends RasiCommands{
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
        Log.d("ROBOTV1COMMANDS", "Driving dist " + dist  + " at angle " + toDegrees(theta));
        driveUtil.drive(dist, theta);
    }

    public void strafe(double dist){
        Log.d("ROBOTV1COMMANDS", "Strafing dist " + dist);
        driveUtil.strafeStraight(dist);
    }

    public void driveStraight(double dist){
        Log.d("ROBOTV1COMMANDS", "DrivingStraight dist" + dist);
        driveUtil.driveStraight(dist, 1);
    }
    public void driveSlow(double dist, double coe){
        Log.d("ROBOTV1COMMANDS", "DrivingStraight dist" + dist);
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

    public void intakeOn(){
        robot.setIntake(true);
    }

    public void reverseIntakeOn() {
        robot.setReverseIntake(true);

    }

    public void intakeOff(){
        robot.setIntake(false);
        robot.setReverseIntake(false);
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
        robot.grab();
        wait(0.5);
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
