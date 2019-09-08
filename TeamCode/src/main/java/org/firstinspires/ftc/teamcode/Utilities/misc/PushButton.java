package org.firstinspires.ftc.teamcode.Utilities.misc;

import com.qualcomm.robotcore.hardware.Gamepad;

public class PushButton extends Button {
    String attr;
    Gamepad gamepad;
    public PushButton(Gamepad gamepad, String Attr){
        super();
        this.attr = Attr;
    }
    public void record(){
        try{
        this.recordNewValue( (boolean) gamepad.getClass().getField(attr).get(gamepad));
        } catch (NoSuchFieldException e){
            throw new RuntimeException("PushButton Doesn't work, apparently (NoSuchFieldException)");
        } catch (IllegalAccessException e){
            throw new RuntimeException("PushButton Doesn't work, apparently (IllegalAccessException)");
        }
    }
}

