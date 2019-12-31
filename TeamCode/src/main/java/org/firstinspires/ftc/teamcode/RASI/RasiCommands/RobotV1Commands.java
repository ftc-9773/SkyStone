package org.firstinspires.ftc.teamcode.RASI.RasiCommands;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        driveUtil.driveStraight(-5.0, 1);

    }

    public void intakeOff(){
        robot.setIntake(false);
        robot.setReverseIntake(false);
    }

    public void vLiftDown(){
        robot.setVLiftPos(0);
    }

    public void vLiftIntakePos() { robot.setVLiftPos(450);}

    public void vLiftRaise(int blocks) { robot.setVLiftPos(blocks*robot.getBlockHeightInEncoders() + 660); }

    public void extendHLift() {robot.extendHLift();}

    public void retractHLift() {robot.retractHLift();}


    public void grab(){
        robot.grab();
        wait(0.5);
        robot.clawOff();
    }

    public void release(){
        robot.release();
        wait(0.5);
        robot.clawOff();
    }

    public void curve(double x, double y, double theta){
        driveUtil.TankCurve(x, y, theta);
    }





}
