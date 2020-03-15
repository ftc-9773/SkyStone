package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.staticRegistrar;

public class MotorController {
    private DcMotorEx motor;
    public double w, a; //rotational velocity (rad/s) and rotational acceleration (rad /s/s)
    public int position;
    boolean trackinfo = true;
    boolean disableEncoder = false;
    DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;


    public MotorController(String name){
        motor = staticRegistrar.opMode.hardwareMap.get(DcMotorEx.class, name);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power){
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }

    //Radians per second.
    public void setVelocity(double velocity){
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocity(velocity, AngleUnit.RADIANS);
    }

    public int getPosition() {
        if (disableEncoder){
            return 0;
        }
        return position;
    }

    public void setRunMode(DcMotor.RunMode runMode){
        if (disableEncoder){
            runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        }
        if (this.runMode != runMode){
            motor.setMode(runMode);
        }
    }
}
