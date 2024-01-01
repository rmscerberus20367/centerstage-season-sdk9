package org.firstinspires.ftc.teamcode.function;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Grip {
    ServoEx gripLeft;
    ServoEx gripRight;
    double leftOpen = 0.5;
    double rightOpen = 0.5;
    double leftClose = 0;
    double rightClose = 0;

    double leftPos;
    double rightPos;



    public void init(HardwareMap hardwareMap){
        gripLeft = new SimpleServo(
                hardwareMap, "gripLeft",  0, 300
        );
        gripRight = new SimpleServo(
                hardwareMap, "gripRight",  0, 300
        );

    }

    public void openLeft() {
        leftPos = leftOpen;
    }
    public void openRight() {
        rightPos = rightOpen;
    }
    public void closeLeft() {
        leftPos = leftClose;
    }
    public void closeRight() {
        rightPos = rightClose;
    }

    public void switchStatesLeft(){
        if (leftPos == leftOpen) closeLeft();
        else if (leftPos == leftClose) openLeft();
    }
    public void switchStatesRight(){
        if (rightPos == rightOpen) closeRight();
        else if (rightPos == rightClose) openRight();
    }
    public void switchStates(){
        switchStatesLeft();
        switchStatesRight();
    }

    public void update(){
        gripLeft.setPosition(leftPos);
        gripRight.setPosition(rightPos);
    }
}
