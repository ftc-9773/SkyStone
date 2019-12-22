package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Controllers;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.LiftDriver;
import org.firstinspires.ftc.teamcode.Utilities.Controllers.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.json.SafeJsonReader;

public class PIDLiftController {
    LiftDriver lift;
    boolean active = true;
    SafeJsonReader reader;
    public int vLiftTargetPos = 0, hLiftTargetPos = 0;
    public double vLiftTargetPow = Double.NaN;

    int minVPosForH;
    int minHPosForLowerV; // Minimum distance needed to extend the horizontal lift to lower the vertical lift passed minVPosForH
    public int minPositiveHPos;

    double clawServoGrabPow, leftClawServoReleasePow, rightClawServoReleasePow;
    double leftServoTargetPow, rightServoTargetPow, rotateServoTargetPos;
    double rotateZeroPos, rotate90Pos, rotate180Pos;
    double capstoneZeroPos, capstoneReleasePos, capstoneTargetPos;

    int blockHeightInEncoders;

    int vLiftMaxPos;
    int hLiftMaxPos;

    public int vliftZeroPos = 0, hliftZeroPos = 0;

    //Height where it is safe to intake blocks
    int vLiftIdlePos;

    private PIDController vpid, hpid;
    double vkp, vkd, vki, hkp, hkd, hki;

    public PIDLiftController(LiftDriver lift){
        this.lift = lift;

        //Get config values
        reader = new SafeJsonReader("RobotV1");

        leftClawServoReleasePow = reader.getDouble("leftClawServoReleasePow");
        clawServoGrabPow = reader.getDouble("clawServoGrabPow");
        rightClawServoReleasePow = reader.getDouble("rightClawServoReleasePow");
        rotateZeroPos = reader.getDouble("rotateServoZeroPos");
        rotate90Pos = reader.getDouble("rotateServo90Pos");
        rotate180Pos = reader.getDouble("rotateServo180Pos");

        vLiftMaxPos = reader.getInt("vLiftMaxPos");
        blockHeightInEncoders = reader.getInt("blockHeightInEncoders");
        vliftZeroPos = lift.getVLiftPos();
        vLiftIdlePos = reader.getInt("vLiftIdlePos");
        hLiftMaxPos = reader.getInt("hLiftMaxPos");
        hliftZeroPos = lift.getHLiftPos();
        minVPosForH = reader.getInt("minHeightForHLift");
        minHPosForLowerV = reader.getInt("minHorizontalToLowerVLIFT");
        minPositiveHPos = minHPosForLowerV;

        capstoneZeroPos = reader.getDouble("capstoneZeroPos");
        capstoneReleasePos = reader.getDouble("capstoneReleasePos");

    }
    //Enable this controller.
    public void activate(){
        active = true;
    }
    public void disable(){
        active = false;
    }

    public void setVLiftPos(int pos) {
        vLiftTargetPos = (int) bound(vliftZeroPos, vLiftMaxPos, pos);
    }

    public void setHLiftPos(int pos){
        hLiftTargetPos = bound(hliftZeroPos, hLiftMaxPos, pos);
    }

    public void update(){
        if (active){

            lift.setClawPow(rightServoTargetPow);
            lift.setRotatePos(rotateServoTargetPos);
        }
    }


    int bound(double min, double max, double v){
        if (v < min){
            return (int)min;
        }
        if (v > max){
            return (int)max;
        }
        return (int)v;
    }
}
