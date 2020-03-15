package org.firstinspires.ftc.teamcode.Converters;

import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Math.Vector;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.SafeJsonReader;
import org.firstinspires.ftc.teamcode.staticRegistrar;

public class VerticalLift extends BaseConverter {
    private double targetPower;
    private int targetPos;
    public int zeroPos;
    public int curPos;
    public int maxPos;
    boolean runningAtPower = false;
    PIDController pid;
    SafeJsonReader json;

    DcMotor liftMotor;

    public VerticalLift(){
        liftMotor = staticRegistrar.opMode.hardwareMap.get(DcMotor.class, "vLiftMotor");
        pid = new PIDController("RobotV1", "v");
        json = new SafeJsonReader("RobotV1");
    }

    //Takes precedence over setTargetPos
    public void setTargetPower(double pow){
        runningAtPower = true;
        targetPower = pow;
    }

    public void setTargetPos(){

    }

    @Override
    public void update() {

    }

}
