package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;


import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.Controllers.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

public class Lifts implements Attachment {
    final String TAG = "Lifts";
    public static final double ENCODER_TICKS_PER_INCH = 560 * (36 / 20 ) * (1/ 2.351); // Obtained by measurement. Units: ticks/rot * rot/rot * rot/inch
    public static final double INCHES_PER_BLOCK_HEIGHT = 4;
    public static final double STUD_HEIGHT_INCHES = 1;
    public final double HEIGHT_OF_PLATFORM_INCHES = 1.25;

    //Physical interface
    DcMotor vLiftMotor, hLiftMotor;
    CRServo leftClawServo, rightClawServo;
    Servo rotateServo;
    Servo capstoneServo;

    //Stores the lowest position the lift should go to. (Which is the initial position of the lifts)
    public int vliftZeroPos = 0, hliftZeroPos = 0;

    SafeJsonReader reader;

    //Possible states
    double leftClawServoGrabPow, rightClawServoGrabPow, leftClawServoReleasePow, rightClawServoReleasePow;
    double leftServoTargetPow, rightServoTargetPow, rotateServoTargetPos;
    double rotateZeroPos, rotate90Pos, rotate180Pos;
    int blockHeightInEncoders;

    int baseLiftHeight, oneBlockHigh, twoBlocksHigh, threeBlocksHigh, fourBlocksHigh, fiveBlocksHigh, sixBlocksHigh, sevenBlocksHigh, eightBlocksHigh, nineBlocksHigh;

    double capstoneZeroPos, capstoneReleasePos, capstoneTargetPos;

    //Safety information, so we don't try to go too high
    int vLiftMaxPos;
    int hLiftMaxPos;

    //Height where it is safe to intake blocks
    int vLiftIdlePos;

    //Stores the target states of the robot. Only one of vLiftTargetPow and vLiftTargetPos can be used at a time. pow is for manual control of the lift.
    public int vLiftTargetPos = 0, hLiftTargetPos = 0;
    public double vLiftTargetPow = Double.NaN;
    int minVPosForH;
    int minHPosForLowerV; // Minimum distance needed to extend the horizontal lift to lower the vertical lift passed minVPosForH
    public int minPositiveHPos;

    private PIDController vpid, hpid;
    double vkp, vkd, vki, hkp, hkd, hki;

    int targetVLiftPos;
    int targetHLiftPos;

    public Lifts(HardwareMap hardwareMap){

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
        hLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Get config values
        reader = new SafeJsonReader("RobotV1");

        leftClawServoGrabPow = reader.getDouble("leftClawServoGrabPow");
        leftClawServoReleasePow = reader.getDouble("leftClawServoReleasePow");
        rightClawServoGrabPow = reader.getDouble("rightClawServoGrabPow");
        rightClawServoReleasePow = reader.getDouble("rightClawServoReleasePow");
        rotateZeroPos = reader.getDouble("rotateServoZeroPos");
        rotate90Pos = reader.getDouble("rotateServo90Pos");
        rotate180Pos = reader.getDouble("rotateServo180Pos");

        vLiftMaxPos = reader.getInt("vLiftMaxPos");
        blockHeightInEncoders = reader.getInt("blockHeightInEncoders");
        vliftZeroPos = getVliftPos();
        vLiftIdlePos = reader.getInt("vLiftIdlePos");
        hLiftMaxPos = reader.getInt("hLiftMaxPos");
        hliftZeroPos = getHLiftPos();
        minVPosForH = reader.getInt("minHeightForHLift");
        minHPosForLowerV = reader.getInt("minHorizontalToLowerVLIFT");
        minPositiveHPos = minHPosForLowerV;

        baseLiftHeight = reader.getInt("baseLiftHeight");
        oneBlockHigh = reader.getInt("oneBlockHigh");
        twoBlocksHigh = reader.getInt("twoBlocksHigh");
        threeBlocksHigh = reader.getInt("threeBlocksHigh");
        fourBlocksHigh = reader.getInt("fourBlocksHigh");
        fiveBlocksHigh = reader.getInt("fiveBlocksHigh");
        sixBlocksHigh = reader.getInt("sixBlocksHigh");
        sevenBlocksHigh = reader.getInt("sevenBlocksHigh");
        eightBlocksHigh = reader.getInt("eightBlocksHigh");
        nineBlocksHigh = reader.getInt("nineBlocksHigh");

        capstoneZeroPos = reader.getDouble("capstoneZeroPos");
        capstoneReleasePos = reader.getDouble("capstoneReleasePos");

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
        rotateServo.setDirection(Servo.Direction.REVERSE);
        capstoneServo.setPosition(capstoneZeroPos);
    }

    //Returns both Hlift and Vlift to state to intake another block.
    public void resetLifts(){
        //setHLiftPos(hliftZeroPos);
        setvLiftPos(vliftZeroPos);
    }

    //In number of blocks. 1 block is 4 inches high, the stud on top is 1 inch.
    public void setvLiftPos(double pos){
        //vLiftTargetPos = (int)bound(vliftZeroPos, vLiftMaxPos, ((pos * INCHES_PER_BLOCK_HEIGHT + STUD_HEIGHT_INCHES + HEIGHT_OF_PLATFORM_INCHES) * ENCODER_TICKS_PER_INCH));
        //vLiftTargetPos = (int)bound(vliftZeroPos, vLiftMaxPos, pos * blockHeightInEncoders + 660);
        Log.d(TAG, "Set pos to " + pos + " calculated from num blocks");

        if (pos == 0.0) { vLiftTargetPos = baseLiftHeight; }
        if (pos == 1.0) { vLiftTargetPos = oneBlockHigh; }
        if (pos == 2.0) { vLiftTargetPos = twoBlocksHigh; }
        if (pos == 3.0) { vLiftTargetPos = threeBlocksHigh; }
        if (pos == 4.0) { vLiftTargetPos = fourBlocksHigh; }
        if (pos == 5.0) { vLiftTargetPos = fiveBlocksHigh; }
        if (pos == 6.0) { vLiftTargetPos = sixBlocksHigh; }
        if (pos == 7.0) { vLiftTargetPos = sevenBlocksHigh; }
        if (pos == 8.0) { vLiftTargetPos = eightBlocksHigh; }
        if (pos == 9.0) { vLiftTargetPos = nineBlocksHigh; }

    }

    public int getvLiftMaxPos(){
        return vLiftMaxPos;
    }

    public int gethLiftMaxPos() { return hLiftMaxPos;}

    public int gethLiftZeroPos() { return hliftZeroPos;}

    public int getBlockHeightInEncoders(){ return blockHeightInEncoders;}

    public int getVliftZeroPos() { return vliftZeroPos;}

    //Set v lift position in terms of encoders.
    public void setvLiftPos(int pos) {
        vLiftTargetPos = (int) bound(vliftZeroPos, vLiftMaxPos, pos);
    }

    //in encoder ticks
    public void setHLiftPos(int pos){
        hLiftTargetPos = (int) bound(hliftZeroPos, hLiftMaxPos, pos);
    }


    public void rotateClaw90(){
        rotateServoTargetPos = rotate90Pos;
    }

    public void rotateClaw180(){
        rotateServoTargetPos = rotate180Pos;
    }

    public void resetClawtoZero(){
        rotateServoTargetPos = rotateZeroPos;
    }

    public void grabBlock(){
            leftServoTargetPow = leftClawServoGrabPow;
            rightServoTargetPow = rightClawServoGrabPow;
            //resetLifts();
    }

    public void stopClaw(){
        leftServoTargetPow = 0;
        rightServoTargetPow = 0;
    }

    public void releaseBlock(){
        rightServoTargetPow = rightClawServoReleasePow;
        leftServoTargetPow = leftClawServoReleasePow;
    }

    private int getRawVlift(){
        return -vLiftMotor.getCurrentPosition();
    }

    private int getRawHLift(){ return -hLiftMotor.getCurrentPosition();}

    public int getVliftPos(){
        return getRawVlift() + vliftZeroPos;
    }

    public int getHLiftPos(){
        return getRawHLift() - hliftZeroPos;
    }

    public void adjustVLift(double pow){
        vLiftTargetPos = (int)bound(vliftZeroPos, vLiftMaxPos, vLiftTargetPos + 100 * pow);

    }

    public void adjustHLift(double pow){
        hLiftTargetPos = (int) bound(hliftZeroPos, hLiftMaxPos, hLiftTargetPos + 10 * pow);
    }

    //Public facing method, with some checks and stuff
    public void setvLiftPow(double pow){
        vLiftTargetPow = pow;
        targetVLiftPos = getVliftPos();
    }

    private void setVLiftPow(double pow){
        vLiftMotor.setPower(pow);
    }

    public void setHLiftPow(double pow){
        hLiftMotor.setPower(pow);
    }

    public void intake(){
        //releaseBlock();
        vLiftTargetPos = vLiftIdlePos;
    }

    public void releaseCapstone(){
        capstoneTargetPos = capstoneReleasePos;
        capstoneServo.setPosition(capstoneTargetPos);
    }

    public void resetCapstone() {
        capstoneTargetPos = capstoneZeroPos;
        capstoneServo.setPosition(capstoneTargetPos);
    }

    @Override
    public void update() {
        double hCorrection = hpid.getPIDCorrection(hLiftTargetPos, getHLiftPos());

        Log.d(TAG, "hcorrection " + hCorrection);
        Log.d(TAG, "Vpos " + getVliftPos());
        Log.d(TAG, "Hpos " + getHLiftPos());
        Log.d(TAG, "vTarget  " + vLiftTargetPos);
        Log.d(TAG, "hTarget " + hLiftTargetPos);

        if (!Double.isNaN(vLiftTargetPow) && false){
            vLiftTargetPos = getVliftPos();
            if (vLiftTargetPow == 0){
                vLiftTargetPow = Double.NaN;
                vLiftTargetPow = 0;
            }
            Log.d(TAG, "VPOWER " + vLiftTargetPow);
            setVLiftPow(vLiftTargetPow);
        } else {
            double vCorrection = vpid.getPIDCorrection(vLiftTargetPos, getVliftPos());
            Log.d(TAG, "VCORRECTION " + vCorrection);
            vCorrection = bound(-1, 1, -vCorrection); //Correct the sign. According to the hardware: counterclockwise = pos, clockwise = neg. Unfortunately, this is backwards with up and down. Therefore, this correction. Same for the vertical lift.
//            if (getVliftPos() - targetVLiftPos < 10 && getVliftPos() - targetVLiftPos > 0){
//                vCorrection = 0;
//            }
            Log.d(TAG, "VCOR2 " + vCorrection);
            setVLiftPow(vCorrection);
        }
        hCorrection = bound(-1, 1, -hCorrection); // Correct the sign
        //setHLiftPow(bound(-1, 1, hCorrection));

        rightClawServo.setPower(rightServoTargetPow);
        leftClawServo.setPower(leftServoTargetPow);
        rotateServo.setPosition(rotateServoTargetPos);
        Log.d(TAG, "Right target pos " + rightServoTargetPow);
        Log.d(TAG, "Right Curr pos " + rightClawServo.getPower());
        Log.d(TAG, "Left target pos " + leftServoTargetPow);
        Log.d(TAG, "Left  Curr pos " + leftClawServo.getPower());
        Log.d(TAG, "Rotate target pos " + rotateServoTargetPos);
        Log.d(TAG, "Rotate Curr pos " + rotateServo.getPosition());

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
