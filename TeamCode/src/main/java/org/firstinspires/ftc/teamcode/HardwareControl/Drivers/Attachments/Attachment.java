package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;

/**
 * Code structure:
 *
 * OpMode → Logic, which changes Robot. Each cycle of the opmode calls Robot.update once, which writes
 * the state of the robot to physical hardware.
 *
 * Attachment is an interface which governs the behavoir of the robots manipulators.(E.g. lifts and stuff)
 *
 * If using these interfaces not through the Robot class, remember to call update each cycle of the opmode.
 * */
public interface Attachment {

    /**
     * Applies target configuration to the hardware
     * */
    void update();

    /**
     *  Sets configuration to a safe stop state.
     * */
    void stop();
}