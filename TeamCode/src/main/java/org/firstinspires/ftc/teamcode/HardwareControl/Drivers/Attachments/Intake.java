package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;
import org.firstinspires.ftc.teamcode.Utilities.misc.Timer;

import java.sql.Driver;
import java.sql.Time;

import android.util.Log;


public class Intake implements Attachment{
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DistanceSensor touchSensor;
    //public DistanceSensor slowFoundation;

    //Private values, keeping track of the current state
    private double pow;
    private double minPow;
    public boolean isOn;
    public boolean loaded = false, slowDown = false;
    public boolean autoOff = false;

    //config values
    private SafeJsonReader reader;

    public double onPow, revPow;
    public Timer stopTimer = null;


    public Intake(HardwareMap hardwareMap){
        reader = new SafeJsonReader("RobotV1");
        minPow = reader.getDouble("minIntakePow", 0.05);
        onPow = reader.getDouble("onIntakePow", 1.0);
        revPow = reader.getDouble("revIntakePow", 0.6);

        leftMotor = hardwareMap.get(DcMotor.class, "lintakeMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rintakeMotor");
        touchSensor = hardwareMap.get(DistanceSensor.class, "intakeTouchSensor");
        //slowFoundation = hardwareMap.get(DistanceSensor.class, "slowFoundation");

        isOn = false;
        off();
        update();
    }

    public void off(){
        this.pow = 0;
        this.isOn = false;
    }

    public void on(){
        this.pow = onPow;
        this.isOn = true;
    }

    public void onReverse(){
        this.pow = -revPow;
        this.isOn = true;
    }

    /**
     * For manually setting the power, if you want to do that for some reason.
     * */
    public void on(double inPow){
        if ((1 >= Math.abs(inPow)) && (Math.abs(inPow) > minPow)){
            this.pow = inPow;
            this.isOn = true;
        }
    }

    public boolean isLoaded(){
        loaded = touchSensor.getDistance(DistanceUnit.CM) <= 26;
        Log.d("INTAKE", "dist " + touchSensor.getDistance(DistanceUnit.CM));
        return loaded;
    }

    public boolean slowDown() {
        //slowDown = (slowFoundation.getDistance(DistanceUnit.CM) <= 25);
        return slowDown;
    }

    public double showFoundDist() {
        //return //slowFoundation.getDistance(DistanceUnit.CM);
        return 0;
    }

    public void setStopTimer(Timer timer){
        stopTimer = timer;
    }

    public void update(){
        if (autoOff && isLoaded() && pow > 0){
            off();
        }
        if (stopTimer != null){
            if (stopTimer.isDone()){
                stopTimer = null;
                off();
            }
        }
        this.leftMotor.setPower(-pow);
        this.rightMotor.setPower(pow);
    }

    public void stop(){
        off();
        update();
        isLoaded();
    }
}
