package org.firstinspires.ftc.teamcode.Opmodes.TeleOpModes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.HardwareControl.Robot;
import org.firstinspires.ftc.teamcode.Utilities.misc.Button;

/**
 * The purpose is to provide a mecanum drivebase teleop drive system, that tracks the position of the robot in inches. Pressing 'a' will
 * reset the posiiton.
 * */
@TeleOp(name = "MecanumDriveTeleOp")
public class MecanumDriveTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivebase drivebase = new MecanumDrivebase(hardwareMap, telemetry);
        Gyro gyro = new Gyro(hardwareMap);
        Robot robot = new Robot(drivebase, gyro, hardwareMap);

        double lastEncoderPos[];
        double currentEncoderPos[];

        double x, y;
        x = 0;
        y = 0;

        double[] readings;
        double r;
        double l;
//        Servo leftHook;
//        leftHook = hardwareMap.get(Servo.class,"leftHook");
//        Servo rightHook;
//        rightHook = hardwareMap.get(Servo.class,"leftHook");

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        OpticalDistanceSensor opticalSensor = hardwareMap.get(OpticalDistanceSensor.class, "opticalSensor");

        Button rb = new Button();
        double hookPosition = 0.0;

        Button a = new Button();

        lastEncoderPos = drivebase.getPos();
        currentEncoderPos = drivebase.getPos();
        waitForStart();
        while(opModeIsActive()){
            a.recordNewValue(gamepad1.a);
            if (a.isJustOn()){
                x = 0;
                y = 0;
            }

            double xp = gamepad1.left_stick_x;
            if (Math.abs(xp) < 0.05){
                xp = 0;
            }
            double yp = gamepad1.left_stick_y;
            if (Math.abs(yp) < 0.05){
                yp = 0;
            }
            drivebase.drive(xp, yp, gamepad1.right_stick_x, true);
            drivebase.update();

            readings = robot.getDistSensorReadings();
            r = readings[0];
            l = readings[1];
            telemetry.addLine("R: " + r + " L: " + l);

            currentEncoderPos = drivebase.getPos();
            x += getDistX(lastEncoderPos, currentEncoderPos);
            y += getDistY(lastEncoderPos, currentEncoderPos);
            y += getDistY(lastEncoderPos, currentEncoderPos);
            telemetry.addLine("X: " + x + " Y: " + y);
            telemetry.addLine("ColorSensor reading red" + colorSensor.red());
            telemetry.addLine("ColorSensor reading blue" + colorSensor.blue());
            telemetry.addLine("ColorSensor reading green" + colorSensor.green());
            telemetry.addLine("OpticalDistSensor reading " + opticalSensor.getRawLightDetected());

            double blue = colorSensor.blue();
            double red = colorSensor.red();
            double green = colorSensor.green();
            telemetry.addLine("red/blue ratio " + blue/red);


            if (blue / red < .60){
                telemetry.addLine("Color: yellow");
            } else {
                telemetry.addLine("Color: black");
            }

            telemetry.update();
            lastEncoderPos = currentEncoderPos;

            boolean hookServos = gamepad1.right_bumper;
            rb.recordNewValue(gamepad1.right_bumper);
            if (rb.isJustOn()) {
                if (hookServos) {
                    if (hookPosition == 0.0) {
                        hookPosition = 0.3;
//                        rightHook.setPosition(hookPosition);
//                        leftHook.setPosition(hookPosition);
                    } else {
                        hookPosition= 0.0;
//                        rightHook.setPosition(hookPosition);
//                        leftHook.setPosition(hookPosition);
                    }
                }

            }
        }
    }

    public double getDistX(double[] last, double[] cur){
        double dx = 0;
        dx += ((cur[0] - last[0]) + (cur[1] - last[1])) / 2;
        return dx / MecanumDrivebase.COUNTS_PER_INCH;
    }
    public double getDistY(double[] last, double[] cur){
        double dx = 0;
        dx += ((cur[0] - last[0]) - (cur[1] - last[1])) / 2;
        return dx / MecanumDrivebase.COUNTS_PER_INCH;
    }


}
