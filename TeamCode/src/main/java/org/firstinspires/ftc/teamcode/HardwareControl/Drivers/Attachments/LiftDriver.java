package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.Controllers.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

public class LiftDriver {
    final String TAG = "LiftDriver";
    public static final double ENCODER_TICKS_PER_INCH = 560 * (36 / 20 ) * (1/ 2.351); // Obtained by measurement. Units: ticks/rot * rot/rot * rot/inch
    public static final double INCHES_PER_BLOCK_HEIGHT = 4;
    public static final double STUD_HEIGHT_INCHES = 1;
    public final double HEIGHT_OF_PLATFORM_INCHES = 1.25;

    DcMotor vLiftMotor, hLiftMotor;
    CRServo leftClawServo, rightClawServo;
    Servo rotateServo;
    Servo capstoneServo;
    SafeJsonReader reader;

    //Per request, they are slides not lifts.
    double vSlidePower;
    double hSlidePower;
    double leftClawServoPow, rightClawServoPow, rotateServoPos, capstoneServoPos;

    public LiftDriver(HardwareMap hardwareMap){

        //Intialise hardware
        vLiftMotor = hardwareMap.get(DcMotor.class, "vLiftMotor");
        hLiftMotor = hardwareMap.get(DcMotor.class, "hLiftMotor");
        leftClawServo = hardwareMap.get(CRServo.class, "leftClawServo");
        rightClawServo = hardwareMap.get(CRServo.class, "rightClawServo");
        rotateServo = hardwareMap.get(Servo.class, "rClawServo");
        capstoneServo = hardwareMap.get(Servo.class, "capServo");

        //Reset encoders
        vLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotateServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setVLiftPower(double pow){
        vSlidePower = pow;
    }

    public void setHSlidePower(double pow){
        hSlidePower = pow;
    }

    public void setCapstonePos(double pos){
        capstoneServoPos = pos;
    }

    public void setRotatePos(double pos){
        rotateServoPos = pos;
    }

    public int getVLiftPos(){
        return vLiftMotor.getCurrentPosition();
    }

    public int getHLiftPos(){
        return hLiftMotor.getCurrentPosition();
    }

    public void setClawPow(double pow){
        leftClawServoPow = pow;
        rightClawServoPow = -pow;
    }

    public void update(){
        vLiftMotor.setPower(vSlidePower);
        hLiftMotor.setPower(hSlidePower);
        leftClawServo.setPower(leftClawServoPow);
        rightClawServo.setPower(rightClawServoPow);
        rotateServo.setPosition(rotateServoPos);
        capstoneServo.setPosition(capstoneServoPos);
    }


}
