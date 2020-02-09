package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;


import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RASI.RasiCommands.RasiCommands;
import org.firstinspires.ftc.teamcode.Utilities.Controllers.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;
import org.firstinspires.ftc.teamcode.Utilities.misc.Timer;
import org.json.JSONException;

public class Lifts implements Attachment {
    final String TAG = "Lifts";
    public static final double ENCODER_TICKS_PER_INCH = 560 * (36 / 20 ) * (1/ 2.351); // Obtained by measurement. Units: ticks/rot * rot/rot * rot/inch
    public static final double INCHES_PER_BLOCK_HEIGHT = 4;
    public static final double STUD_HEIGHT_INCHES = 1;
    public final double HEIGHT_OF_PLATFORM_INCHES = 1.25;
    public final double GEAR_RATIO = 28.0/28.0;
    public int RESTING_POSITION_TO_ZERO_OFFSET = 0; //Encoder ticks.

    public boolean disablePIDLiftControl = false;

    //Physical interface
    DcMotor vLiftMotor;
    //DcMotor hLiftMotor;
    CRServo leftClawServo, rightClawServo;
    Servo rotateServo;
    Servo capstoneServo;
    Servo hLiftServo;
    DigitalChannel magLimitSwitch;

    //Stores the lowest position the lift should go to. (Which is the initial position of the lifts)
    public int vliftZeroPos = 0, hliftZeroPos = 0;

    SafeJsonReader reader;

    //Possible states
    double leftClawServoGrabPow, rightClawServoGrabPow, leftClawServoReleasePow, rightClawServoReleasePow;
    double leftServoTargetPow, rightServoTargetPow, rotateServoTargetPos;
    double rotateZeroPos, rotate90Pos, rotate180Pos;
    int blockHeightInEncoders;

    int baseLiftHeight, oneBlockHigh, twoBlocksHigh, threeBlocksHigh, fourBlocksHigh, fiveBlocksHigh, sixBlocksHigh, sevenBlocksHigh, eightBlocksHigh, nineBlocksHigh, tenBlocksHigh;

    double capstoneZeroPos, capstoneReleasePos, capstoneTargetPos;

    double hLiftServoZeroPos, hLiftServoExtendPos, hLiftServoTargetPos;

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

    private PIDController vpidUp, vpidDown, hpid;
    double vkpu, vkdu, vkiu, vkpd, vkdd, vkid;

    int targetVLiftPos;
    double correctionFactor = 0;
    int targetHLiftPos;
    int error;

    public class LiftLowerer{
        int lastPos;
        long lastTime;
        double speed;
        public boolean done = false;
        public LiftLowerer(){
            lastPos = vLiftMotor.getCurrentPosition();
            lastTime = System.currentTimeMillis();
            setVLiftPow(0.1);
        }
        public void update(){
            if (done){
                return;
            }
            setVLiftPow(0.1);
            speed = (vLiftMotor.getCurrentPosition()- lastPos) / (System.currentTimeMillis() - lastTime);
            if (readLimitSwitch() || speed < 0.01){
                stop();
            }
        }
        public void stop(){
            done = true;
        }
    }

    public Lifts(HardwareMap hardwareMap){

        //Intialise hardware
        vLiftMotor = hardwareMap.get(DcMotor.class, "vLiftMotor");
        //hLiftMotor = hardwareMap.get(DcMotor.class, "hLiftMotor");
        leftClawServo = hardwareMap.get(CRServo.class, "leftClawServo");
        rightClawServo = hardwareMap.get(CRServo.class, "rightClawServo");
        rotateServo = hardwareMap.get(Servo.class, "rClawServo");
        capstoneServo = hardwareMap.get(Servo.class, "capServo");
        hLiftServo = hardwareMap.get(Servo.class, "hLiftServo");
        magLimitSwitch = hardwareMap.get(DigitalChannel.class, "magLimitSwitch");

        //Reset encoders
        vLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //hLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //hLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Get config values
        reader = new SafeJsonReader("RobotV1");

        int RESTING_POSITION_TO_ZERO_OFFSET_;
        RESTING_POSITION_TO_ZERO_OFFSET = reader.getInt("EncoderOffset");
        leftClawServoGrabPow = reader.getDouble("leftClawServoGrabPow");
        leftClawServoReleasePow = reader.getDouble("leftClawServoReleasePow");
        rightClawServoGrabPow = reader.getDouble("rightClawServoGrabPow");
        rightClawServoReleasePow = reader.getDouble("rightClawServoReleasePow");
        rotateZeroPos = reader.getDouble("rotateServoZeroPos");
        rotate90Pos = reader.getDouble("rotateServo90Pos");
        rotate180Pos = reader.getDouble("rotateServo180Pos");

        vLiftMaxPos = reader.getInt("vLiftMaxPos") + RESTING_POSITION_TO_ZERO_OFFSET;
        blockHeightInEncoders = reader.getInt("blockHeightInEncoders");
        retractHLift();
        vLiftMotor.setPower(0.4);
        while (readLimitSwitch()){
            Log.d(TAG, "Limit switch reading " + readLimitSwitch());
        }
        vLiftMotor.setPower(0);
        Timer timer = new Timer(0.5);
        while (!timer.isDone()){
            Log.d(TAG, "waiting for done");
        }

        vLiftIdlePos = reader.getInt("vLiftIdlePos") + RESTING_POSITION_TO_ZERO_OFFSET;
        hLiftMaxPos = reader.getInt("hLiftMaxPos");
        //hliftZeroPos = getHLiftPos();
        minVPosForH = reader.getInt("minHeightForHLift") + RESTING_POSITION_TO_ZERO_OFFSET;
        minHPosForLowerV = reader.getInt("minHorizontalToLowerVLIFT") + RESTING_POSITION_TO_ZERO_OFFSET;
        minPositiveHPos = minHPosForLowerV;

        baseLiftHeight = reader.getInt("baseLiftHeight") + RESTING_POSITION_TO_ZERO_OFFSET;
        oneBlockHigh = reader.getInt("oneBlockHigh") + RESTING_POSITION_TO_ZERO_OFFSET;
        twoBlocksHigh = reader.getInt("twoBlocksHigh") + RESTING_POSITION_TO_ZERO_OFFSET;
        threeBlocksHigh = reader.getInt("threeBlocksHigh") + RESTING_POSITION_TO_ZERO_OFFSET;
        fourBlocksHigh = reader.getInt("fourBlocksHigh") + RESTING_POSITION_TO_ZERO_OFFSET;
        fiveBlocksHigh = reader.getInt("fiveBlocksHigh") + RESTING_POSITION_TO_ZERO_OFFSET;
        sixBlocksHigh = reader.getInt("sixBlocksHigh") + RESTING_POSITION_TO_ZERO_OFFSET;
        sevenBlocksHigh = reader.getInt("sevenBlocksHigh") + RESTING_POSITION_TO_ZERO_OFFSET;
        eightBlocksHigh = reader.getInt("eightBlocksHigh") + RESTING_POSITION_TO_ZERO_OFFSET;
        nineBlocksHigh = reader.getInt("nineBlocksHigh") + RESTING_POSITION_TO_ZERO_OFFSET;
        tenBlocksHigh = reader.getInt("tenBlocksHigh") + RESTING_POSITION_TO_ZERO_OFFSET;

        capstoneZeroPos = reader.getDouble("capstoneZeroPos");
        capstoneReleasePos = reader.getDouble("capstoneReleasePos");

        hLiftServoExtendPos = reader.getDouble("hLiftServoExtend");
        hLiftServoZeroPos = reader.getDouble("hLiftServoZero");


        //Set up pids.
        vkpu = reader.getDouble("vkpu");
        vkdu = reader.getDouble("vkdu");
        vkiu = reader.getDouble("vkiu");
        vpidUp = new PIDController(vkpu, vkiu, vkdu);
        vkpd = reader.getDouble("vkpd");
        vkdd = reader.getDouble("vkdd");
        vkid = reader.getDouble("vkid");
        vpidDown = new PIDController(vkpd, vkid, vkdd);

        vliftZeroPos = getRawVlift() - RESTING_POSITION_TO_ZERO_OFFSET;
        rotateServoTargetPos = rotateZeroPos;
        rotateServo.setDirection(Servo.Direction.REVERSE);
        capstoneServo.setPosition(capstoneZeroPos);
        hLiftServo.setPosition(hLiftServoZeroPos);
        retractHLift();
    }

    public boolean readLimitSwitch(){
        return magLimitSwitch.getState();
    }

    public void writeZeroToJson(){
        try {
        reader.jsonRoot.put("vLiftZeroPos", vliftZeroPos);
        } catch (JSONException e){
            Log.e(TAG, "", e);
        }
    }
    public void readZeroPos(){
        vliftZeroPos = reader.getInt("vLiftZeroPos");
    }

    public void setCorrectionFactor(double factor){
        correctionFactor = factor;
    }

    //Returns both Hlift and Vlift to state to intake another block.
    public void resetLifts(){
        //setHLiftPos(hliftZeroPos);
        setvLiftPos(vliftZeroPos);
    }

    public double getGearRatio() {
        return GEAR_RATIO;
    }

    //In number of blocks. 1 block is 4 inches high, the stud on top is 1 inch.
    public void setvLiftPos(double pos){
        //vLiftTargetPos = (int)bound(vliftZeroPos, vLiftMaxPos, ((pos * INCHES_PER_BLOCK_HEIGHT + STUD_HEIGHT_INCHES + HEIGHT_OF_PLATFORM_INCHES) * ENCODER_TICKS_PER_INCH));
        //vLiftTargetPos = (int)bound(vliftZeroPos, vLiftMaxPos, pos * blockHeightInEncoders + 660);
        Log.d(TAG, "Set pos to " + pos + " calculated from num blocks");

        if (pos == 0.0) { vLiftTargetPos = (int)(baseLiftHeight * GEAR_RATIO); }
        if (pos == 1.0) { vLiftTargetPos = (int)(oneBlockHigh * GEAR_RATIO); }
        if (pos == 2.0) { vLiftTargetPos = (int)(twoBlocksHigh * GEAR_RATIO); }
        if (pos == 3.0) { vLiftTargetPos = (int)(threeBlocksHigh * GEAR_RATIO); }
        if (pos == 4.0) { vLiftTargetPos = (int)(fourBlocksHigh * GEAR_RATIO); }
        if (pos == 5.0) { vLiftTargetPos = (int)(fiveBlocksHigh * GEAR_RATIO); }
        if (pos == 6.0) { vLiftTargetPos = (int)(sixBlocksHigh * GEAR_RATIO); }
        if (pos == 7.0) { vLiftTargetPos = (int)(sevenBlocksHigh * GEAR_RATIO); }
        if (pos == 8.0) { vLiftTargetPos = (int)(eightBlocksHigh * GEAR_RATIO); }
        if (pos == 9.0) { vLiftTargetPos = (int)(nineBlocksHigh * GEAR_RATIO); }
        if (pos == 10.0) { vLiftTargetPos = (int)(tenBlocksHigh * GEAR_RATIO); }

    }

    public int getvLiftMaxPos(){
        return vLiftMaxPos;
    }

    //public int gethLiftMaxPos() { return hLiftMaxPos;}

    //public int gethLiftZeroPos() { return hliftZeroPos;}

    public int getBlockHeightInEncoders(){ return blockHeightInEncoders;}

    public int getVliftZeroPos() { return vliftZeroPos;}

    //Set v lift position in terms of encoders.
    public void setvLiftPos(int pos) {
        vLiftTargetPos = (int) bound(vliftZeroPos, vLiftMaxPos, pos);
    }

    //in encoder ticks
//    public void setHLiftPos(int pos){
//        hLiftTargetPos = (int) bound(hliftZeroPos, hLiftMaxPos, pos);
//    }


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

    //private int getRawHLift(){
    // return -hLiftMotor.getCurrentPosition();}

    public int getVliftPos(){
        return getRawVlift() - vliftZeroPos;
    }

//    public int getHLiftPos(){
//        return getRawHLift() - hliftZeroPos;
//    }

    public void adjustVLift(double pow){
        vLiftTargetPos = (int)bound(vliftZeroPos, vLiftMaxPos, vLiftTargetPos + 100 * pow);

    }

//    public void adjustHLift(double pow){
//        hLiftTargetPos = (int) bound(hliftZeroPos, hLiftMaxPos, hLiftTargetPos + 10 * pow);
//    }

    //Public facing method, with some checks and stuff
    public void setvLiftPow(double pow){
        vLiftTargetPow = pow;
        targetVLiftPos = getVliftPos();
    }

    private void setVLiftPow(double pow){
        vLiftMotor.setPower(pow);
    }

//    public void setHLiftPow(double pow){
//        hLiftMotor.setPower(pow);
//    }

    public void extendHLift() {
        hLiftServoTargetPos = hLiftServoExtendPos;
        hLiftServo.setPosition(hLiftServoTargetPos);
    }

    public void retractHLift() {
        hLiftServoTargetPos = hLiftServoZeroPos;
        hLiftServo.setPosition(hLiftServoTargetPos);
    }

    public int getIntakePos() { return vLiftIdlePos;}

    public void intake(){
        //releaseBlock();
        vLiftTargetPos = (int)(vLiftIdlePos * GEAR_RATIO);
    }

    public void releaseCapstone(){
        capstoneTargetPos = capstoneReleasePos;
        capstoneServo.setPosition(capstoneTargetPos);
    }

    public void resetCapstone() {
        capstoneTargetPos = capstoneZeroPos;
        capstoneServo.setPosition(capstoneTargetPos);
    }

    public void disablePIDLiftControl(){
        targetVLiftPos = vliftZeroPos;
        disablePIDLiftControl = true;
    }

    public void enablePIDLiftControl(){
        targetVLiftPos = vliftZeroPos = getRawVlift();
        disablePIDLiftControl = false;
    }

    @Override
    public void update() {
        Log.d(TAG, "Vpos " + getVliftPos());
        Log.d(TAG, "vTarget  " + vLiftTargetPos);
        Log.d(TAG, "hTarget " + hLiftTargetPos);

        error = vLiftTargetPos - getVliftPos();

        double vCorrection;
        if (error > 0) vCorrection = vpidUp.getPIDCorrection(error);
        else vCorrection = vpidDown.getPIDCorrection(error);

        Log.d(TAG, "VCORRECTION " + vCorrection);
        vCorrection = bound(-1, 1, -vCorrection); //Correct the sign. According to the hardware: counterclockwise = pos, clockwise = neg. Unfortunately, this is backwards with up and down. Therefore, this correction. Same for the vertical lift.

        Log.d(TAG, "VCOR2 " + vCorrection);
//        if(correctionFactor != 0){
//            if (vCorrection < correctionFactor){
//                vCorrection = correctionFactor;
//            }
////        }
        if (vLiftTargetPos == 0 && getVliftPos() < 20){
            vCorrection = 0;
        }
        if (!disablePIDLiftControl) setVLiftPow(vCorrection);

        if (hLiftServoTargetPos == hLiftServoZeroPos) {
            retractHLift();
        }
        else {
            extendHLift();
        }
        hLiftServo.setPosition(hLiftServoTargetPos);
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
