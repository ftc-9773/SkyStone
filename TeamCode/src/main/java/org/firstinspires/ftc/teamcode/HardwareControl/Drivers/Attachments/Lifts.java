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
    public static final double INCHES_PER_BLOCK_HIGHT = 4;
    public static final double STUD_HIGHT_INCHES = 1;
    public final double HIGHT_OF_PLATFORM_INCHES = 1.25;

    DcMotor vLiftMotor, hLiftMotor;
    Servo leftClawServo, rightClawServo, rotateServo;

    int vliftZeroPos = 0, hliftZeroPos = 0;

    SafeJsonReader reader;

    double leftClawServoGrabPos, rightClawServoGrabPos, leftClawServoReleasePos, rightClawServoReleasePos;
    double leftServoTargetPos, rightServoTargetPos, rotateServoTargetPos;
    double rotateZeroPos, rotate90Pos, rotateneg90Pos;

    int vLiftMaxPos;
    int hLiftMaxPos;
    int vLiftIdlePos;
    int vLiftTargetPos, hLiftTargetPos;
    int minVPosForH;
    int minHPosForLowerV; // Minimum distance needed to extend the horizontal lift to lower the vertical lift passed minVPosForH

    public double minPositiveHPos;

    private PIDController vpid, hpid;
    double vkp, vkd, vki, hkp, hkd, hki;

    //For coordinating actions that must be done in sequence (e.g. can't retract h lift when v lifts is all the way down)
    boolean waitingToGrabBlock = false;
    public boolean retractingHLift = false;
    public boolean loweringVLift = false;
    public boolean raisingVLift = false;
    public double futureTargetHLiftPos;

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
        rotateZeroPos = reader.getDouble("rotateServoZeroPos");
        rotate90Pos = reader.getDouble("rotateServo90Pos");
        rotateneg90Pos = reader.getDouble("rotateServoneg90Pos");

        vLiftMaxPos = reader.getInt("vLiftMaxPos");
        vliftZeroPos = getVliftPos();
        vLiftIdlePos = reader.getInt("vLiftIdlePos");
        hLiftMaxPos = reader.getInt("hLiftMaxPos");
        hliftZeroPos = getHLiftPos();
        minVPosForH = reader.getInt("minHeightForHLift");
        minHPosForLowerV = reader.getInt("minHorizontalToLowerVLIFT");
        minPositiveHPos = minHPosForLowerV;

        //Set up pids.
        vkp = reader.getDouble("vkp");
        vkd = reader.getDouble("vkd");
        vki = reader.getDouble("vki");
        vpid = new PIDController(vkp, vki, vkd);
        hkp = reader.getDouble("hkp");
        hkd = reader.getDouble("hkd");
        hki = reader.getDouble("hki");
        hpid = new PIDController(hkp, hki, hkd);

        rotateServoTargetPos = rotateZeroPos;

    }

    //Returns both Hlift and Vlift to state to intake another block.
    public void vLiftDown(){
        loweringVLift = true;
        if ((rotateServoTargetPos - rotateZeroPos > 10)){
            resetClawtoZero();
        } else if(!hLiftIsRetracted()){
            hLiftDown();
        } else {
            vLiftTargetPos = vLiftIdlePos;
        }
    }

    public void hLiftDown(){
        if (getVliftPos() > minVPosForH){
            retractingHLift = true;
            setvLiftPos(minVPosForH);
        } else {
            hLiftTargetPos = hliftZeroPos;
            retractingHLift = false;
        }
    }

    //In number of blocks.
    public void setvLiftPos(double pos){
        vLiftTargetPos = (int)bound(vliftZeroPos, vLiftMaxPos, ((pos * INCHES_PER_BLOCK_HIGHT + STUD_HIGHT_INCHES + HIGHT_OF_PLATFORM_INCHES) * ENCODER_TICKS_PER_INCH));
    }

    //Set v lift position in terms of encoders.
    private void setvLiftPos(int pos) {
        vLiftTargetPos = (int) bound(vliftZeroPos, vLiftMaxPos, pos);
    }

    //Right now, in encoder ticks
    public void sethLiftPos(double pos){
        if (getVliftPos() < minVPosForH && pos < minHPosForLowerV) {
            setvLiftPos(minVPosForH);
            raisingVLift = true;
            futureTargetHLiftPos = pos;
        } else {
            hLiftTargetPos = (int) pos;
            raisingVLift = false;
        }
    }

    public void rotateClaw(double speed){
        rotateServoTargetPos = bound(rotateneg90Pos, rotate90Pos, rotateServoTargetPos + 0.01 * speed);
    }
    /**
     * @param direction: either -1 or 1 depending on which way you want to rotate 90 degrees.
     * */
    public void rotateClaw90(double direction){
        rotateServoTargetPos = bound(rotateneg90Pos, rotate90Pos, rotateServoTargetPos + direction * (rotate90Pos + rotateneg90Pos) / 2);
    }
    public void resetClawtoZero(){
        rotateServoTargetPos = rotateZeroPos;
    }

    public void grabBlock(){
        if(getVliftPos() > vliftZeroPos){
            vLiftTargetPos = vliftZeroPos;
            waitingToGrabBlock = true;
        } else {
            leftServoTargetPos = leftClawServoGrabPos;
            rightServoTargetPos = rightClawServoGrabPos;
            waitingToGrabBlock = false;
            vLiftDown();
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

    public boolean hLiftIsRetracted(){
        return (getHLiftPos() - hliftZeroPos) < 10;
    }

    public void adjustVLift(double pow){
        if (hLiftIsRetracted() ||  minHPosForLowerV < getHLiftPos()){
            vLiftTargetPos = (int)bound(vliftZeroPos, vLiftMaxPos, vLiftTargetPos + 10 * pow);
        }
    }

    public void adjustHLift(double pow){
        if (hLiftTargetPos < minHPosForLowerV && pow > 0){
            hLiftTargetPos = minHPosForLowerV;
            return;
        } else if (hLiftTargetPos < minHPosForLowerV) {
            hLiftTargetPos = hliftZeroPos;
        }
        hLiftTargetPos = (int) bound(hliftZeroPos, hLiftMaxPos, hLiftTargetPos + 10 * pow);
        if (getHLiftPos() > minHPosForLowerV && hLiftTargetPos < minHPosForLowerV){
            hLiftTargetPos = hliftZeroPos;
        }
    }

    private void setVLiftPow(double pow){
        vLiftMotor.setPower(pow);
    }

    private void setHLiftPow(double pow){
        if (getVliftPos() > minVPosForH){
        hLiftMotor.setPower(pow);
        }
    }

    @Override
    public void update() {
        double vCorrection = vpid.getPIDCorrection(vLiftTargetPos, getVliftPos());
        double hCorrection = hpid.getPIDCorrection(hLiftTargetPos, getHLiftPos());

        if (waitingToGrabBlock){grabBlock();}
        if (retractingHLift) {hLiftDown();}
        if (loweringVLift) {vLiftDown();}
        if (raisingVLift) {sethLiftPos(futureTargetHLiftPos);}

        setVLiftPow(bound(-1, 1, vCorrection));
        setHLiftPow(bound(-1, 1, hCorrection));

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
