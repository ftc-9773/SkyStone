package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.Controllers.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.Controllers.VelocityController;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

public class Lifts implements Attachment {
    final String TAG = "Lifts";
    public static final double ENCODER_TICKS_PER_INCH = 560 * (36 / 20 ) * (1/ 2.351); // Obtained by measurement. Units: ticks/rot * rot/rot * rot/inch
    public static final double INCHES_PER_BLOCK_HEIGHT = 4;
    public static final double STUD_HEIGHT_INCHES = 1;
    public final double HEIGHT_OF_PLATFORM_INCHES = 1.25;

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
    public int vLiftTargetPos = 0, hLiftTargetPos = 0;
    int minVPosForH;
    int minHPosForLowerV; // Minimum distance needed to extend the horizontal lift to lower the vertical lift passed minVPosForH
    boolean magical_boolean = false;
    public int minPositiveHPos;

    private PIDController vpid, hpid;
    //private PIDController vpid, hpid;
    double vkp, vkd, vki, hkp, hkd, hki;
    double va, ha, vMaxV, hMaxV, vMinPow, hMinPow;


    /**
     * For coordinating when doing stuff (auton, not teleOp. TeleOp just doesn't work if it's an invalid state):
     *
     * lowering vertical Lift:
     *  - IF Vlift.DOWN -> Lower, everything should be fine. Double check HLIFT to see if something went very wrong
     *  - IF Vlift.UP -> Check Hlift
     *  - - IF Hlift.OUT or Hlift.IN -> lower Vlift, everything should be fine.
     *  - - IF Hlift.MID -> extend or retract hlift. If it is in the back 1/3 -> retract otherwise extend to hlift.OUT
     *  raising vertical lift:
     *  - Shouldn't every cause problems. TODO: check to make sure it never causes problems
     *  extending Horizontal Lift:
     *  - IF Hlift.OUT -> won't cause problems, because Hlift is already out.
     *  - IF Hlift.IN -> Check Vlift
     *  - - If Vlift.DOWN -> raise Vlift to UP, then extend Hlift
     *  - - If Vlift.UP -> No problems will occur (probably)
     *  retracting Horizontal Lift
     *   - IF VLift.UP || Hlift.IN -> No problems will occur.
     *   - IF VLift.DOWN && HLift.OUT && Target pos <= HLift.MID -> Raise VLift to UP before retracting
     *   - IF Vlift.DOWN && HLift.OUT && Target pos > Hlift.MID -> Retract lift, it won't cause problems.
     *
     *  Action Meanings:
     *   - VLift.RAISE = Raise the vertical lift as soon as the state of the robot permits it. In the meantime, take actions to move the robot to a valid state.
     *   - VLift.LOWER = Lower the vertical lift as soon as the state of the robot permits it. In the meantime, take actions to move the robot to a valid state.
     *   - VLift.NONE =  No actions are queued for the vertical lift. Note: this does NOT mean that the vertical lift is not moving. Just that it is safe for the vertical lift to do what it is doing.
     *   (horizontal actions mean analogous things for the HLift)
     *  All Queued actions use variables like targetSubAttachementPos to specify parameters when the queued action is executed.
     *  e.g targetHLiftPos
     * */
    enum VerticalLiftStates {UP, DOWN} // UP = High enough to retract the horizontal lift from all the way out to in. DOWN = Not high enough to……
    enum HorizontalLiftStates {OUT, MID, IN} // OUT = Far enough out to lower Vertical lift to DOWN pos, MID = Can't lower V lift past DOWN pos, IN = All the way in, can lower vLIft
    enum VerticalLiftActions {RAISE, LOWER, NONE} //What is currently happening to the vLift. Keeps track, if actions have to happen in sequence.
    enum HorizontalLiftActions {EXTEND, RETRACT, NONE} //What is currently happening to the hLift

    public VerticalLiftActions vLiftAction = VerticalLiftActions.NONE;
    public VerticalLiftStates vLiftState = VerticalLiftStates.DOWN;
    public HorizontalLiftActions hLiftAction = HorizontalLiftActions.NONE;
    public HorizontalLiftStates hLiftState  = HorizontalLiftStates.IN;
    boolean waitingToGrabBlock = false;
    int targetVLiftPos;
    int targetHLiftPos;

    public Lifts(HardwareMap hardwareMap){

        //Intialise hardware
        vLiftMotor = hardwareMap.get(DcMotor.class, "vLiftMotor");
        hLiftMotor = hardwareMap.get(DcMotor.class, "hLiftMotor");
        leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");
        rotateServo = hardwareMap.get(Servo.class, "rotateServo");

        //Reset encoders
        vLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        Log.d(TAG, "Zero Pos " + vliftZeroPos);
        Log.d(TAG, "Target Pos " + vLiftTargetPos);
        Log.d(TAG, "Vertical Pos " + getVliftPos());

    }

    //Returns both Hlift and Vlift to state to intake another block.
    public void resetLifts(){
        checkHliftState();
        checkVLiftState();
        if (hLiftState != HorizontalLiftStates.IN){
            vLiftAction = VerticalLiftActions.LOWER;
            targetVLiftPos = vliftZeroPos;
            setHLiftPos(hliftZeroPos); // TODO: Might lead to overwriting vertical lift queued positions. Probably doesn't. Rewrite whole system in terms of queued actions.
        } else {
            setvLiftPos(vliftZeroPos);
        }
    }

    //Update and return the state of the hLift
    public HorizontalLiftStates checkHliftState(){
        if (getHLiftPos() <= hliftZeroPos + 10){
            hLiftState = HorizontalLiftStates.IN;
        } else if (getHLiftPos() > hliftZeroPos && getHLiftPos() < minHPosForLowerV){
            hLiftState = HorizontalLiftStates.MID;
        } else if (getHLiftPos() > minHPosForLowerV){
            hLiftState = HorizontalLiftStates.OUT;
        } else {
            Log.e(TAG, "An invalid Horizontal Lift position was reached with position " + getHLiftPos());
        }
        return hLiftState;
    }
    //Update and return the state of the vLift
    public VerticalLiftStates checkVLiftState(){
        if (getVliftPos() <= minVPosForH){
            vLiftState = VerticalLiftStates.DOWN;
            Log.d(TAG, "State of Vertical Lift: " + vLiftState);
        } else if (getVliftPos() > minVPosForH){
            vLiftState = VerticalLiftStates.UP;
            Log.d(TAG, "State of Vertical Lift: " + vLiftState);
        } else {
            Log.e(TAG, "An invalid Horizontal Lift position was reached with position " + getHLiftPos());
        }
        return vLiftState;
    }


    //In number of blocks. 1 block is 4 inches high, the stud on top is 1 inch.
    public void setvLiftPos(double pos){
        vLiftTargetPos = (int)bound(vliftZeroPos, vLiftMaxPos, ((pos * INCHES_PER_BLOCK_HEIGHT + STUD_HEIGHT_INCHES + HEIGHT_OF_PLATFORM_INCHES) * ENCODER_TICKS_PER_INCH));
        Log.d(TAG, "Set pos to " + pos + " calculated from num blocks");
    }

    //Set v lift position in terms of encoders.
    private void setvLiftPos(int pos) {
        vLiftTargetPos = (int) bound(vliftZeroPos, vLiftMaxPos, pos);
    }

    //Right now, in encoder ticks
    public void setHLiftPos(int pos){
        checkHliftState();
        if (vLiftState == VerticalLiftStates.DOWN){
            if (hLiftState.equals(HorizontalLiftStates.IN) && pos > minPositiveHPos){
                setvLiftPos(minVPosForH);
                hLiftAction =  pos < getHLiftPos()?  HorizontalLiftActions.RETRACT : HorizontalLiftActions.EXTEND;
                targetHLiftPos = pos;
                return;
            } else if (hLiftState != HorizontalLiftStates.IN && pos < minHPosForLowerV){
                setvLiftPos(minVPosForH);
                hLiftAction =  pos < getHLiftPos()? HorizontalLiftActions.RETRACT : HorizontalLiftActions.EXTEND;
                targetHLiftPos = pos;
                return;
            }
        } else {
            hLiftTargetPos = (int) bound(hliftZeroPos, hLiftMaxPos, pos);
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
        if(getVliftPos() >= vliftZeroPos && false){
            vLiftTargetPos = vliftZeroPos;
            waitingToGrabBlock = true;
        } else {
            leftServoTargetPos = leftClawServoGrabPos;
            rightServoTargetPos = rightClawServoGrabPos;
            waitingToGrabBlock = false;
            //resetLifts();
        }
    }

    public void releaseBlock(){
        waitingToGrabBlock = false;
        rightServoTargetPos = rightClawServoReleasePos;
        leftServoTargetPos = leftClawServoReleasePos;
    }

    private int getRawVlift(){
        return -vLiftMotor.getCurrentPosition();
    }

    private int getRawHLift(){ return -hLiftMotor.getCurrentPosition();}

    public int getVliftPos(){
        return getRawVlift() + vliftZeroPos;
    }

    public int getHLiftPos(){
        return getRawHLift();
    }

    public boolean hLiftIsRetracted(){
        return (getHLiftPos() - hliftZeroPos) < 10;
    }

    public void adjustVLift(double pow){
        if (hLiftIsRetracted() ||  minHPosForLowerV < getHLiftPos()){
            vLiftTargetPos = (int)bound(vliftZeroPos, vLiftMaxPos, vLiftTargetPos + 50 * pow);
        }
    }

    public void adjustHLift(double pow){
        hLiftTargetPos = (int) bound(hliftZeroPos, hLiftMaxPos, hLiftTargetPos + 10 * pow);
    }

    public void setVLiftPow(double pow){
        vLiftMotor.setPower(pow);
    }

    public void setHLiftPow(double pow){
        hLiftMotor.setPower(pow);
    }

    @Override
    public void update() {
        double vCorrection = vpid.getPIDCorrection(vLiftTargetPos, getVliftPos());
        double hCorrection = hpid.getPIDCorrection(hLiftTargetPos, getHLiftPos());

        Log.d(TAG, "VCORRECTION " + vCorrection);
        Log.d(TAG, "hcorrection " + hCorrection);
        Log.d(TAG, "Vpos " + getVliftPos());
        Log.d(TAG, "Hpos " + getHLiftPos());
        Log.d(TAG, "vTarget  " + vLiftTargetPos);
        Log.d(TAG, "hTarget " + hLiftTargetPos);

        if (waitingToGrabBlock){grabBlock();}
        if (hLiftAction == HorizontalLiftActions.EXTEND && getVliftPos() > minVPosForH){
            hLiftTargetPos = targetHLiftPos;
        }
        if (hLiftAction == HorizontalLiftActions.RETRACT && getVliftPos() > minVPosForH){
            hLiftTargetPos = targetHLiftPos;
        }
        if (vLiftAction == VerticalLiftActions.LOWER && (getHLiftPos() > minHPosForLowerV || getHLiftPos() <= hliftZeroPos + 10)){
            vLiftTargetPos = targetVLiftPos;
        }

        vCorrection = bound(-1, 1, -vCorrection); //Correct the sign. According to the hardware: counterclockwise = pos, clockwise = neg. Unfortunately, this is backwards with up and down. Therefore, this correction. Same for the vertical lift.
        if (getVliftPos() - targetVLiftPos < 10 && getVliftPos() - targetVLiftPos > 0){
            vCorrection = 0;
        }
        hCorrection = bound(-1, 1, -hCorrection); // Correct the sign
        setVLiftPow(vCorrection);
        //setHLiftPow(bound(-1, 1, hCorrection));

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
