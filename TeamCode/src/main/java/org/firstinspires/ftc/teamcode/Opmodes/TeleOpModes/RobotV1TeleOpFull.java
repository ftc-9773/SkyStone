package org.firstinspires.ftc.teamcode.Opmodes.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.BackHooks;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Intake;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.Lifts;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments.SideHook;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Drivebase.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Sensors.Gyro;
import org.firstinspires.ftc.teamcode.HardwareControl.RobotV1;

@TeleOp(name = "RobotTeleOPFull")
public class RobotV1TeleOpFull extends LinearOpMode {

    @Override
    public void runOpMode(){
        MecanumDrivebase drivebase = new MecanumDrivebase(hardwareMap, telemetry);
        sendTelemetry("Drivebase created...");

        Intake intake = new Intake(hardwareMap);
        sendTelemetry("Intake created...");

        Gyro gyro = new Gyro(hardwareMap);
        sendTelemetry("Gyro created...");

        Lifts lifts = new Lifts(hardwareMap);
        sendTelemetry("Lifts created...");

        BackHooks backHooks = new BackHooks(hardwareMap);
        sendTelemetry("Back Hooks interface created...");

        SideHook sideHook = new SideHook(hardwareMap);

        RobotV1 robot = new RobotV1(drivebase, gyro, intake, lifts, backHooks, telemetry, sideHook);
        sendTelemetry("Robot created...");


        sendTelemetry("Waiting for start...");
        robot.drivebase.runWithoutEncoders();

        // opmode start
        waitForStart();
        boolean temp = true;
        long lastTime = System.currentTimeMillis();
        while(opModeIsActive()) {
            if (temp){
                sendTelemetry("Started");
                temp = false;
            }

            robot.runGamepadCommands(gamepad1, gamepad2);
            robot.update();
            // interested in seeing cycle time
            long currTime = System.currentTimeMillis();
            telemetry.addLine("Lift Height Y: " + robot.yLiftHeight + "  B: " + robot.bLiftHeight);
            telemetry.addLine("Current V Pos: " + lifts.getVliftPos());
            telemetry.addLine("Target V Pos: " + lifts.vLiftTargetPos);
            telemetry.addLine("Current H Pos: "  + lifts.getHLiftPos());
            telemetry.addLine("Target H Pos: " + lifts.hLiftTargetPos);
            telemetry.addLine("Intake Loaded  " + intake.isLoaded());
            telemetry.addData("Elapsed Time Milliseconds: ", (currTime - lastTime));
            lastTime = currTime;

            telemetry.update();
        }

    }
    private void sendTelemetry(String msg){
        telemetry.addLine(msg);
        telemetry.update();
    }
}
