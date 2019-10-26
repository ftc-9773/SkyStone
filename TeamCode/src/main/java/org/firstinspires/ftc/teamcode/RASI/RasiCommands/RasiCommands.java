package org.firstinspires.ftc.teamcode.RASI.RasiCommands;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class for defining functions that can be used in RASI. As of version 2.5, there is no way to
 * define functions inside a RASI file.
 *
 * //TODO Create detailed guides and documentation for RASI
 * For a detailed guide on RASI, please see RASI/guides
 *
 * The return type of the function does not matter, and it is best to leave it as void. As of version
 * 2.5, there is no way to store information in a RASI script, all variables should be contained and accessed
 * in this class.
 *
 * RASI is not case sensitive, therefore CreateRobot() and CrEaTeRoBot() will cause errors. Use whichever
 * is more readable. RASI has not been tested with conflicting function names, unknown errors may occur.
 *
 * Reserved function names and their uses:
 * - addTag      : Add the following argument as a tag for this programs execution
 * - endOpMode   : This command ends the LinearOpMode, as if the user pressed the stop button on the device
 * - removeTag   : This command removes a previously added tag from the possible tags during this programs execution.
 *      The tag can still be added back later.
 * Possible future function names. While they will cause not issues yet, they should not be used to prevent future incompatibilty
 * - addTags    : Works as addTag except that it adds all arguments as tags
 * - removeTags : Works as removeTag except that it removes all arguments from the Tag list
 * - end        : Terminates the execution of the RASI file
 *
 * Note: Rasi 2.5 is not fully backwards compatible with version 2.0. In particular, end has been replaced by endOpMode and semicolons
 * are not used as statement terminators. Statements must be written on one line, and end with a newline. Otherwise,
 * all other functions should work.
 *
 * NOTE: RASI 2.5 DOES NOT SUPPORT FUNCTIONS WITH STRING ARGUMENTS. THIS INCLUDES THE TAG MANIPULATION FUNCTIONS.
 * ----Please add documentation to the functions you create-----
 * @author cadence
 * @version 2.5
 * */

public class RasiCommands {
    private LinearOpMode opMode;
    private Telemetry telemetry;


    public RasiCommands(LinearOpMode o){
        this.opMode = o;
        this.telemetry = o.telemetry;
    }

    public void stop(){
        opMode.requestOpModeStop();
    }

    public void printAll(String[] words){
        StringBuilder msg = new StringBuilder();
        for (String word : words){
            msg.append(word);
        }
        telemetry.addLine(msg.toString());
        telemetry.update();
    }

    public void write(String msg){
        telemetry.addLine(msg);
        telemetry.update();
    }
    /**
     * Stops executing for a duration of time
     * @param timeInSeconds The amount of seconds to pause execution
     * */


}
