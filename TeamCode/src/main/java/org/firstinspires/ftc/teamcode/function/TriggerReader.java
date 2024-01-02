package org.firstinspires.ftc.teamcode.function;



public class TriggerReader {
    Boolean lastcycle = false;
    KeyReader trigger = new KeyReader();

    public boolean wasJustPressed(double key){
        boolean keybool = false;
        if (key>=0.3){keybool = true;}

        return trigger.wasJustPressed(keybool);

    }
}
