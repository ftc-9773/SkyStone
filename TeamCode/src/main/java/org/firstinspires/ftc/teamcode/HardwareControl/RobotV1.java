package org.firstinspires.ftc.teamcode.HardwareControl;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Intake;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Lifts;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;

public class RobotV1 extends Robot {
    double x = 0, y = 0;

    Intake intake;
    Lifts lifts;

    public RobotV1(MecanumDrivebase drivebase, Gyro gryo, Intake intake, Lifts lifts){
        this.lifts = lifts;
        this.intake = intake;
        this.gyro = gyro;
        this.drivebase = drivebase;
    }

    public void runGamepadCommands(Gamepad gamepad1, Gamepad gamepad2){
        drivebase.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, true);
        if (gamepad2.left_bumper){
            intake.on();
        } else if (gamepad2.left_trigger > 0.05){
            intake.on(gamepad2.left_trigger);
        }

        if (gamepad2.dpad_down){
            lifts.vLiftDown();
        }
        if (gamepad2.b){
            lifts.hLiftDown();
        }

        if (Math.abs(gamepad2.left_stick_y) > 0.05){
            lifts.setVLiftPow(gamepad2.left_stick_y);
        }
        if (Math.abs(gamepad2.right_stick_y) > 0.05){
            lifts.setHLiftPow(gamepad2.right_stick_y);
        }
        if (gamepad2.a){
            lifts.releaseBlock();
        }
        if (gamepad2.x){
            lifts.grabBlock();
        }

    }

    public void setIntake(boolean on){
        if (on) intake.on();
        else intake.off();
    }


    @Override
    public void update() {
        drivebase.update();
        intake.update();
        heading = getHeading();
    }

    public void stop(){}
}
