package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.Controllers.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

public class Lifts implements Attachment {
    final String TAG = "Lifts";
    public static final double ENCODER_TICKS_PER_INCH = 540; // Obtained by measurement

    DcMotor vLiftMotor, hLiftMotor;
    Servo leftClawServo, rightClawServo, rotateServo;

    int vliftZeroPos = 0, hliftZeroPos = 0;

    SafeJsonReader reader;

    double leftClawServoGrabPos, rightClawServoGrabPos, leftClawServoReleasePos, rightClawServoReleasePos;
    double leftServoTargetPos, rightServoTargetPos;
    double vLiftMaxPos;
    double hLiftMaxPos;
    int vLiftTargetPos, hLiftTargetPos;
    double minVPosForH;

    private PIDController vpid, hpid;
    double vkp, vkd, vki, hkp, hkd, hki;

    boolean waitingToGrabBlock = false;

    public Lifts(HardwareMap hardwareMap){

        //Intialise hardware
        vLiftMotor = hardwareMap.get(DcMotor.class, "vLiftMotor");
        hLiftMotor = hardwareMap.get(DcMotor.class, "hLiftMotor");
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");

        //Get config values
        reader = new SafeJsonReader("RobotV1");

        leftClawServoGrabPos = reader.getDouble("leftClawServoGrabPos");
        leftClawServoReleasePos = reader.getDouble("leftClawServoReleasePos");
        rightClawServoGrabPos = reader.getDouble("rightClawServoGrabPos");
        rightClawServoReleasePos = reader.getDouble("rightClawServoReleasePos");

        vLiftMaxPos = reader.getDouble("vLiftMaxPos");
        vliftZeroPos = getVliftPos();
        hLiftMaxPos = reader.getDouble("hLiftMaxPos");
        hliftZeroPos = getHLiftPos();
        minVPosForH = reader.getDouble("minHightForHLift");


        //Set up pids.
        vkp = reader.getDouble("vkp");
        vkd = reader.getDouble("vkd");
        vki = reader.getDouble("vki");
        vpid = new PIDController(vkp, vki, vkd);
        hkp = reader.getDouble("hkp");
        hkd = reader.getDouble("hkd");
        hki = reader.getDouble("hki");
        hpid = new PIDController(hkp, hki, hkd);

    }

    public void vLiftDown(){
        vLiftTargetPos = vliftZeroPos;
    }

    public void hLiftDown(){hLiftTargetPos = hliftZeroPos;}

    //In number of blocks. TODO: Write to take the input in the form of blocks, rather than encoder ticks.
    public void setvLiftPos(int pos){
        vLiftTargetPos = pos;
    }

    //Number of inches would be ideal here. Right now, in encoder ticks
    public void sethLiftPos(double pos){
        hLiftTargetPos = (int) pos;
    }

    public void grabBlock(){
        if(getVliftPos() > 20){
            vLiftDown();
            waitingToGrabBlock = true;
        } else {
            leftServoTargetPos = leftClawServoGrabPos;
            rightServoTargetPos = rightClawServoGrabPos;
            waitingToGrabBlock = false;
        }
    }

    public void releaseBlock(){
        waitingToGrabBlock = false;
        rightServoTargetPos = rightClawServoReleasePos;
        leftServoTargetPos = leftClawServoReleasePos;
    }

    public int getVliftPos(){
        return vLiftMotor.getCurrentPosition() - vliftZeroPos;
    }

    public int getHLiftPos(){
        return hLiftMotor.getCurrentPosition() - hliftZeroPos;
    }

    //Caution, this should not be used publically. Temporary.
    public void setVLiftPow(double pow){
        vLiftMotor.setPower(pow);
    }
    //Caution, this should not be used publically. Temporary.
    public void setHLiftPow(double pow){
        if (getVliftPos() > minVPosForH){
        hLiftMotor.setPower(pow);
        }
    }

    @Override
    public void update() {
        double vCorrection = vpid.getPIDCorrection(vLiftTargetPos, getVliftPos());
        double hCorrection = hpid.getPIDCorrection(hLiftTargetPos, getHLiftPos());

        setVLiftPow(bound(-1, 1, vCorrection));
        setHLiftPow(bound(-1, 1, hCorrection));

        if (waitingToGrabBlock){grabBlock();}

        rightClawServo.setPosition(rightServoTargetPos);
        leftClawServo.setPosition(leftServoTargetPos);
    }

    @Override
    public void stop() {

    }

    double bound(double min, double max, double v){
        if (v < min){
            return min;
        }
        if (v > max){
            return max;
        }
        return v;
    }
}
