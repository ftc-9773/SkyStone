package org.firstinspires.ftc.teamcode.HardwareControl.Drivers.Attachments;

public interface Attachment {

    /**
     * Applies target configuration
     * */
    void update();

    /**
     *  Sets configuration to a safe stop state
     * */
    void stop();

    /**
     * Checks whether the attachment is in the state that it should be
     */
    boolean inStableState();

}