package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

import java.sql.Driver;

public class Intake implements Attachment{
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DistanceSensor touchSensor;
    public Servo autoServo;

    //Private values, keeping track of the current state
    private double pow;
    private double minPow;
    public boolean isOn;
    public boolean loaded = false;
    public double autoServoDownPos, autoServoUpPos;

    //config values
    private SafeJsonReader reader;

    public double onPow;


    public Intake(HardwareMap hardwareMap){
        reader = new SafeJsonReader("RobotV1");
        minPow = reader.getDouble("minIntakePow", 0.05);
        onPow = reader.getDouble("onIntakePow", 1.0);
        autoServoDownPos = reader.getDouble("autoServoDownPos");
        autoServoUpPos = reader.getDouble("autoServoUpPos");

        leftMotor = hardwareMap.get(DcMotor.class, "lintakeMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rintakeMotor");
        touchSensor = hardwareMap.get(DistanceSensor.class, "intakeTouchSensor");
        autoServo = hardwareMap.get(Servo.class, "autoServo");

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
        this.pow = -onPow;
        this.isOn = true;
    }

    /**
     * For manually setting the power, if you want to do that for some reason.
     * */
    public void on(double inPow){
        if ((1 >= inPow) && (inPow > minPow)){
            this.pow = inPow;
            this.isOn = true;
        }
    }

    public void autoServoDown() {
        autoServo.setPosition(autoServoDownPos);
    }

    public void autoServoUp() {
        autoServo.setPosition(autoServoUpPos);
    }

    public boolean isLoaded(){
        loaded = touchSensor.getDistance(DistanceUnit.CM) <= 3;
        return loaded;
    }

    public void update(){
        this.leftMotor.setPower(-pow);
        this.rightMotor.setPower(pow);
    }

    public void stop(){
        off();
        update();
        isLoaded();
    }
}
