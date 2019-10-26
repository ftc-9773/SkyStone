package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

import java.sql.Driver;

public class Intake implements Attachment{
    public DcMotor leftMotor;
    public DcMotor rightMotor;

    //Private values, keeping track of the current state
    private double pow;
    private double minPow;
    public boolean isOn;

    //config values
    private SafeJsonReader reader;

    public double onPow;


    public Intake(HardwareMap hardwareMap){
        reader = new SafeJsonReader("RobotV1");
        minPow = reader.getDouble("minIntakePow", 0.05);
        onPow = reader.getDouble("onIntakePow", 1.0);

        leftMotor = hardwareMap.get(DcMotor.class, "lintakeMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rintakeMotor");

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

    /**
     * For manually setting the power, if you want to do that for some reason.
     * */
    public void on(double inPow){
        if ((1 >= inPow) && (inPow > minPow)){
            this.pow = inPow;
            this.isOn = true;
        }
    }

    public void update(){
        this.leftMotor.setPower(pow);
        this.rightMotor.setPower(-pow);
    }

    public void stop(){
        off();
        update();
    }
}
