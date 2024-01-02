package org.firstinspires.ftc.teamcode.function;

import com.qualcomm.robotcore.hardware.Gamepad;

public class KeyReader {
    Boolean lastcycle = false;


    public boolean wasJustPressed(Boolean key){
        if (key == true && key != lastcycle){
            lastcycle = key;
            return true;
        }
        lastcycle = key;
        return false;
    }


}
