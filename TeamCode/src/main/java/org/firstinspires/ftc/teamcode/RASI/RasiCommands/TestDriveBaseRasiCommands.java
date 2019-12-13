package org.firstinspires.ftc.teamcode.RASI.RasiCommands;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareControl.Robot;
import org.firstinspires.ftc.teamcode.Logic.DriveUtil;

public class TestDriveBaseRasiCommands extends RasiCommands {
    DriveUtil driveUtil;
    Robot robot;

    public TestDriveBaseRasiCommands(LinearOpMode o, Robot r){
        super(o);
        this.robot = r;
        driveUtil = new DriveUtil(robot, o);
    }

    public void driveStraight(double dist){
        Log.d("ROBOTV1COMMANDS", "DrivingStraight dist" + dist);
        driveUtil.driveStraight(dist, 1);
    }

    public void wait(double time){
        long startTime = System.currentTimeMillis();
        while(startTime + time*1000 > System.currentTimeMillis() && !opMode.isStopRequested()){
            robot.update();
        }
    }
}
