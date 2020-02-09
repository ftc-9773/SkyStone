package org.firstinspires.ftc.teamcode.HardwareControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;

public class Driver {
    public Robot robot;
    public MecanumDrivebase drivebase;
    public LinearOpMode opMode;
    VoltageSensor voltageSensor;


    public Driver(Robot robot, LinearOpMode opMode){
        this.robot = robot;
        drivebase = robot.drivebase;
        this.opMode = opMode;
    }



    /**
     * Returns voltage
     * @param angVel Angular velocity in radians per second.
     *
     * */
    public double getVoltageFromAngVelocity(double angVel){
        return 0;
    }
}
